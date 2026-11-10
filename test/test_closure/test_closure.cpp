#include "Utils/ConfigRecord.hpp"
#include "Connect/MspProcessor.hpp"
#include "Connect/MspParser.hpp"
#include "Connect/Cli.hpp"
#include "Blackbox/BlackboxBridge.h"
#include "Blackbox/BlackboxSerialBuffer.h"
#include "Connect/MspBoxMapping.hpp"
#include "Connect/Vtx.hpp"
#include "Output/Mixer.h"
#include "Control/RescueSettings.hpp"
#include "Control/GpsRescue.h"
#include "Sensor/VoltageSensor.hpp"
#include "msp/msp_protocol.h"
#include "Utils/Crc.hpp"
#include <ArduinoFake.h>
#include <unity.h>
#include <vector>
#include <deque>
#include <cstring>
using namespace Espfc;
using namespace fakeit;
static unsigned long clockUs;
void setUp() {
 ArduinoFakeReset(); clockUs=1000000;
 When(Method(ArduinoFake(), micros)).AlwaysDo([](){return clockUs;});
 When(Method(ArduinoFake(), millis)).AlwaysDo([](){return clockUs/1000;});
 EscDriverBase::testBeginSucceeds=true; EscDriverBase::testFailAttachChannel=-1;
 EscDriverBase::testEndCalls=0;
}
void tearDown() { ArduinoFakeReset(); }
class TestWire: public Espfc::Stream::ReadWritable {
public:
 std::deque<uint8_t> rx; std::vector<uint8_t> tx; size_t maxWrite=255; int space=255; bool drained=true; int baud=4800;
 void begin(const Hal::SerialDeviceConfig&) override {}
 void updateBaudRate(int b) override {baud=b;}
 int available() override {return rx.size();}
 int read() override {if(rx.empty())return -1;int x=rx.front();rx.pop_front();return x;}
 int peek() override {return rx.empty()?-1:rx.front();}
 size_t readMany(uint8_t* p,size_t n) override {size_t k=0;while(k<n&&!rx.empty())p[k++]=read();return k;}
 size_t write(uint8_t b) override {return write(&b,1);}
 size_t write(const uint8_t* p,size_t n) override {n=std::min(n,maxWrite);tx.insert(tx.end(),p,p+n);return n;}
 int availableForWrite() override {return space;}
 void flush() override {TEST_FAIL_MESSAGE("VTX must never block on flush");}
 bool isTxFifoEmpty() override {return drained;}
 void reply(uint8_t cmd,std::initializer_list<uint8_t> p,bool bad=false) {
  std::vector<uint8_t> frame={cmd,uint8_t(p.size()+1)};frame.insert(frame.end(),p.begin(),p.end());
  frame.push_back(Utils::crc8_dvb_s2(0,frame.data(),frame.size()) ^ (bad?1:0));
  rx.push_back(0xaa);rx.push_back(0x55);for(uint8_t b:frame)rx.push_back(b);
 }
};
static Connect::MspResponse process(Model& model,uint16_t cmd,const std::vector<uint8_t>& payload={}) {
 Connect::MspProcessor msp(model); TestWire wire; Connect::MspMessage message;message.cmd=cmd;
 message.version=Connect::MSP_V2;message.received=payload.size();std::copy(payload.begin(),payload.end(),message.buffer);
 Connect::MspResponse r;msp.processCommand(message,r,wire);return r;
}
void test_record_roundtrip_and_every_byte_corruption_rejected() {
 ModelConfig c,d; c.extension.capacityMah=2200;std::vector<uint8_t> record(Utils::ConfigRecord::HEADER+sizeof(c));
 TEST_ASSERT_TRUE(Utils::ConfigRecord::encode(c,record.data(),record.size()));
 TEST_ASSERT_TRUE(Utils::ConfigRecord::decode(record.data(),record.size(),d));
 TEST_ASSERT_EQUAL_UINT16(2200,d.extension.capacityMah);
 for(size_t i=0;i<record.size();++i) {record[i]^=1;d.extension.capacityMah=111;
  TEST_ASSERT_FALSE(Utils::ConfigRecord::decode(record.data(),record.size(),d));
  TEST_ASSERT_EQUAL_UINT16(111,d.extension.capacityMah);record[i]^=1;}
}
void test_record_rejects_wrong_layout_and_noncanonical_bool_even_with_valid_crc() {
 ModelConfig c,d;std::vector<uint8_t> r(Utils::ConfigRecord::HEADER+sizeof(c));
 TEST_ASSERT_TRUE(Utils::ConfigRecord::encode(c,r.data(),r.size()));r[8]^=1;
 Utils::ConfigRecord::recordWord(r.data()+12,~Utils::ConfigRecord::crc(Utils::ConfigRecord::crc(0xffffffff,r.data(),12),r.data()+16,sizeof(c)),4);
 TEST_ASSERT_FALSE(Utils::ConfigRecord::decode(r.data(),r.size(),d));
 TEST_ASSERT_TRUE(Utils::ConfigRecord::encode(c,r.data(),r.size()));
 r[16+offsetof(ModelConfig,input)+offsetof(InputConfig,filterEnable)]=2;
 Utils::ConfigRecord::recordWord(r.data()+12,~Utils::ConfigRecord::crc(Utils::ConfigRecord::crc(0xffffffff,r.data(),12),r.data()+16,sizeof(c)),4);
 TEST_ASSERT_FALSE(Utils::ConfigRecord::decode(r.data(),r.size(),d));
}
void test_record_invalid_config_is_not_encoded() {
 ModelConfig c; uint8_t r[2048]; c.output.motorPoles=0;
 TEST_ASSERT_FALSE(Utils::ConfigRecord::encode(c,r,sizeof(r)));
}
void test_msp_api46_shapes_and_roundtrips() {
 Model m;struct Pair{uint16_t get,set;unsigned n;};
 for(Pair p: {Pair{MSP_RX_CONFIG,MSP_SET_RX_CONFIG,38},Pair{MSP_RC_TUNING,MSP_SET_RC_TUNING,23},
  Pair{MSP_FILTER_CONFIG,MSP_SET_FILTER_CONFIG,49},Pair{MSP_RX_MAP,MSP_SET_RX_MAP,8},Pair{MSP_BATTERY_CONFIG,MSP_SET_BATTERY_CONFIG,13}}) {
  auto r=process(m,p.get);TEST_ASSERT_EQUAL_INT(1,r.result);TEST_ASSERT_EQUAL_UINT(p.n,r.len);
  auto s=process(m,p.set,std::vector<uint8_t>(r.data,r.data+r.len));TEST_ASSERT_EQUAL_INT_MESSAGE(1,s.result,"Configurator roundtrip rejected");
 }
 auto api=process(m,MSP_API_VERSION);TEST_ASSERT_EQUAL_UINT8(46,api.data[2]);
 auto version=process(m,MSP_FC_VERSION);TEST_ASSERT_EQUAL_UINT(3,version.len);
}
void test_msp_all_truncated_stock_rescue_packets_are_atomic() {
 Model m; auto r=process(m,MSP_GPS_RESCUE);
#if defined(ESPFC_ALTHOLD_V2_ACTIVE) && defined(ESPFC_LAND_V2_ACTIVE)
 TEST_ASSERT_EQUAL_UINT(26,r.len);const ModelConfig before=m.config;
 for(unsigned n=1;n<26;++n) {auto s=process(m,MSP_SET_GPS_RESCUE,{r.data,r.data+n});
  TEST_ASSERT_EQUAL_INT(-1,s.result);TEST_ASSERT_EQUAL_MEMORY(&before,&m.config,sizeof(before));}
 TEST_ASSERT_EQUAL_INT(1,process(m,MSP_SET_GPS_RESCUE,{r.data,r.data+26}).result);
 auto read=process(m,MSP_GPS_RESCUE);TEST_ASSERT_EQUAL_MEMORY(r.data,read.data,26);
 auto custom=process(m,Connect::MSP2_ESPFC_GPS_RESCUE_CONFIG);TEST_ASSERT_EQUAL_INT(-1,custom.result);
#else
 TEST_ASSERT_EQUAL_INT(-1,r.result);
#endif
}
void test_msp_invalid_composite_battery_write_is_atomic() {
 Model m; auto r=process(m,MSP_BATTERY_CONFIG);ModelConfig before=m.config;
 std::vector<uint8_t> p(r.data,r.data+r.len);p[7]=0xf4;p[8]=1; // minimum 5V exceeds maximum
 TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_BATTERY_CONFIG,p).result);
 TEST_ASSERT_EQUAL_MEMORY(&before,&m.config,sizeof(before));
}
void test_msp_battery_thresholds_capacity_persist_through_record() {
 Model m; auto r=process(m,MSP_BATTERY_CONFIG);std::vector<uint8_t> p(r.data,r.data+r.len);
 p[3]=0x98;p[4]=8; // 2200mAh
 p[7]=0x4a;p[8]=1; // min330
 p[9]=0xa4;p[10]=1; // max420
 p[11]=0x5e;p[12]=1; // warn350
 TEST_ASSERT_EQUAL_INT(1,process(m,MSP_SET_BATTERY_CONFIG,p).result);
 uint8_t record[2048];ModelConfig loaded;
 TEST_ASSERT_TRUE(Utils::ConfigRecord::encode(m.config,record,sizeof(record)));
 TEST_ASSERT_TRUE(Utils::ConfigRecord::decode(record,sizeof(record),loaded));
 TEST_ASSERT_EQUAL_UINT16(330,loaded.extension.cellMinimum);TEST_ASSERT_EQUAL_UINT16(2200,loaded.extension.capacityMah);
}
void test_serial_policy_rejects_multiple_uart_consumers() {
 SerialPortConfig p; p.functionMask=SERIAL_FUNCTION_GPS|SERIAL_FUNCTION_MSP;
 TEST_ASSERT_FALSE(Utils::serialPortValid(p));p.functionMask=SERIAL_FUNCTION_BLACKBOX;p.blackboxBaud=0;
 TEST_ASSERT_FALSE(Utils::serialPortValid(p));p.blackboxBaud=250000;TEST_ASSERT_TRUE(Utils::serialPortValid(p));
}
void test_mixer_failed_begin_or_attach_never_becomes_ready() {
#ifndef ESPFC_SAFE_BENCH_BUILD
 Model m; m.config.output.protocol=ESC_PROTOCOL_PWM;Output::Mixer mixer(m);
 EscDriverBase::testBeginSucceeds=false;TEST_ASSERT_EQUAL_INT(0,mixer.begin());TEST_ASSERT_FALSE(m.state.mixer.outputReady);
 EscDriverBase::testBeginSucceeds=true;EscDriverBase::testFailAttachChannel=2;
 TEST_ASSERT_EQUAL_INT(0,mixer.begin());TEST_ASSERT_FALSE(m.state.mixer.outputReady);TEST_ASSERT_NULL(m.state.mixer.escMotor);
 TEST_ASSERT_GREATER_THAN_UINT(0,EscDriverBase::testEndCalls);TEST_ASSERT_EQUAL_INT(0,mixer.update());
 EscDriverBase::testFailAttachChannel=-1;TEST_ASSERT_EQUAL_INT(1,mixer.begin());TEST_ASSERT_TRUE(m.state.mixer.outputReady);
#endif
}
void test_mixer_empty_invalid_or_duplicate_outputs_inhibit_arming() {
 Model m; m.config.output.protocol=ESC_PROTOCOL_PWM;Output::Mixer mixer(m);m.config.mixer.type=127;
 TEST_ASSERT_EQUAL_INT(0,mixer.begin());TEST_ASSERT_TRUE(m.armingDisabled());
 m.config.mixer.type=FC_MIXER_QUADX;m.config.pin[PIN_OUTPUT_1]=m.config.pin[PIN_OUTPUT_0];
#ifndef ESPFC_SAFE_BENCH_BUILD
 TEST_ASSERT_EQUAL_INT(0,mixer.begin());TEST_ASSERT_FALSE(m.state.mixer.outputReady);
#endif
}
void test_stock_rescue_altitude_modes_and_min_sats() {
 // Controller adapter tests below establish wire storage; this pure unit checks min-sats fault qualification.
 ModelConfig c;c.extension.stockRescueEnabled=1;c.extension.stockRescue.minSats=9;
 TEST_ASSERT_EQUAL_UINT(9,Control::rescueMinimumSatellites(c));
 c.extension.stockRescueEnabled=0;c.gps.minSats=7;TEST_ASSERT_EQUAL_UINT(7,Control::rescueMinimumSatellites(c));
 c.gpsRescue.altitudeMode=3;StockRescueSettings wire;TEST_ASSERT_FALSE(Control::stockRescueSettings(c,wire));
}
void test_vtx_partial_write_preserves_frame_and_crc() {
 Model m;TestWire wire;wire.maxWrite=1;Connect::Vtx v(m);v.begin(&wire);v.update();
 for(unsigned n=0;n<6;++n) v.update();
 const uint8_t expected[]={0,0xaa,0x55,3,0,0x9f};
 TEST_ASSERT_EQUAL_UINT(6,wire.tx.size());TEST_ASSERT_EQUAL_MEMORY(expected,wire.tx.data(),6);
}
static void discover(Connect::Vtx& v,TestWire& w) {
 v.update();v.update();v.update();w.reply(1,{7,7,0,0x16,0xe9});v.update();
}
void test_vtx_wrong_crc_and_unrelated_ack_do_not_report_ready() {
 Model m;TestWire w;Connect::Vtx v(m);v.begin(&w);v.update();v.update();v.update();
 w.reply(1,{7,7,0,0x16,0xe9},true);v.update();TEST_ASSERT_FALSE(m.state.vtx.active);TEST_ASSERT_EQUAL_UINT(1,m.state.vtx.crcErrors);
 w.reply(3,{7});v.update();TEST_ASSERT_FALSE(m.state.vtx.active);
 w.reply(1,{7,7,0,0x16,0xe9});v.update();TEST_ASSERT_TRUE(m.state.vtx.active);TEST_ASSERT_TRUE(m.state.vtx.ready);
}
void test_vtx_applied_settings_wait_for_matching_ack() {
 Model m;TestWire w;Connect::Vtx v(m);v.begin(&w);discover(v,w);TEST_ASSERT_TRUE(m.state.vtx.ready);
 m.config.vtx.channel=1;v.update();v.update();v.update();
 w.reply(3,{7});v.update();TEST_ASSERT_EQUAL_UINT(8,m.state.vtx.appliedChannel);TEST_ASSERT_FALSE(m.state.vtx.ready);
 w.reply(3,{0});v.update();TEST_ASSERT_EQUAL_UINT(1,m.state.vtx.appliedChannel);TEST_ASSERT_TRUE(m.state.vtx.ready);
}
void test_vtx_backpressure_and_timeout_are_bounded() {
 Model m;TestWire w;w.space=0;Connect::Vtx v(m);v.begin(&w);v.update();
 for(unsigned i=0;i<3;++i){clockUs+=300001;v.update();}
 TEST_ASSERT_EQUAL_UINT(0,w.tx.size());TEST_ASSERT_FALSE(m.state.vtx.ready);
 clockUs+=1000001;v.update();TEST_ASSERT_EQUAL_INT(4900,w.baud);
}
void test_vtx_v21_uses_advertised_dbm_power() {
 Model m;TestWire w;Connect::Vtx v(m);v.begin(&w);v.update();v.update();v.update();
 w.reply(17,{7,0,0,0x16,0xe9,14,2,0,14,20});v.update();
 TEST_ASSERT_EQUAL_UINT(3,m.state.vtx.version);TEST_ASSERT_EQUAL_UINT(2,m.state.vtx.powerLevels);
 m.config.vtx.power=2;v.update();v.update();v.update();
 TEST_ASSERT_EQUAL_UINT8(0x94,w.tx[w.tx.size()-2]);
 w.reply(2,{20});v.update();TEST_ASSERT_EQUAL_UINT(2,m.state.vtx.appliedPower);TEST_ASSERT_TRUE(m.state.vtx.ready);
}
void test_msp_decode_error_does_not_poison_the_next_frame() {
 Connect::MspMessage m; m.readU16(); TEST_ASSERT_TRUE(m.decodeError);
 Connect::MspParser parser;const uint8_t packet[]={'$','M','<',0,MSP_API_VERSION,1};for(uint8_t b : packet) parser.parse(b,m);
 TEST_ASSERT_TRUE(m.isReady());TEST_ASSERT_FALSE(m.decodeError);
}
void test_cli_unsigned_short_preserves_capacity_range() {
 uint16_t value=1000; Connect::Cli::Param p("capacity",&value);
 const char* full[]={"set","capacity","65535",nullptr};TEST_ASSERT_TRUE(p.update(full));TEST_ASSERT_EQUAL_UINT16(65535,value);
 const char* negative[]={"set","capacity","-1",nullptr};TEST_ASSERT_FALSE(p.update(negative));TEST_ASSERT_EQUAL_UINT16(65535,value);
 const char* overflow[]={"set","capacity","65536",nullptr};TEST_ASSERT_FALSE(p.update(overflow));TEST_ASSERT_EQUAL_UINT16(65535,value);
}
void test_msp_invalid_failsafe_does_not_turn_into_drop() {
 Model m; m.config.failsafe.procedure=FAILSAFE_PROCEDURE_AUTO_LAND;
 auto get=process(m,MSP_FAILSAFE_CONFIG);std::vector<uint8_t> p(get.data,get.data+get.len);p[7]=255;
 TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_FAILSAFE_CONFIG,p).result);
 TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PROCEDURE_AUTO_LAND,m.config.failsafe.procedure);
}
void test_msp_status_api46_remaps_arm_switch_and_has_exact_length() {
 Model m;m.state.mode.armingDisabledFlags=ARMING_DISABLED_ARM_SWITCH|ARMING_DISABLED_MOTOR_PROTOCOL;
 auto r=process(m,MSP_STATUS);TEST_ASSERT_EQUAL_UINT16(24,r.len);TEST_ASSERT_EQUAL_UINT8(26,r.data[16]);
 uint32_t bits=uint32_t(r.data[17])|(uint32_t(r.data[18])<<8)|(uint32_t(r.data[19])<<16)|(uint32_t(r.data[20])<<24);
 TEST_ASSERT_EQUAL_HEX32((1u<<25)|(1u<<24),bits);
}
void test_blackbox_close_progresses_without_optional_sample_updates() {
 TestWire wire;serialDeviceInit(&wire,0);targetPidLooptime=250;currentPidProfile=pidProfilesMutable(0);
 blackboxConfigMutable()->blackbox_uart=SERIAL_PORT_USART1;blackboxConfigMutable()->blackbox_baud=BAUD_115200;blackboxConfigMutable()->device=BLACKBOX_DEVICE_SERIAL;blackboxConfigMutable()->mode=BLACKBOX_MODE_NORMAL;
 blackboxConfigMutable()->sample_rate=1;armingFlags=ARMED;blackboxInit();blackboxUpdate(clockUs);
 TEST_ASSERT_FALSE(blackboxMayEditConfig());blackboxFinish();TEST_ASSERT_TRUE(blackboxIsShuttingDown());
 wire.drained=false;TEST_ASSERT_FALSE(blackboxServiceShutdown());
 clockUs+=1000000;TEST_ASSERT_TRUE(blackboxServiceShutdown());TEST_ASSERT_TRUE(blackboxMayEditConfig());
 TEST_ASSERT_TRUE(blackboxShutdownWasTruncated());armingFlags=0;serialDeviceInit(nullptr,0);
}
void test_blackbox_drained_close_and_lifecycle_byte_budget() {
 TestWire wire;Blackbox::BlackboxSerialBuffer b;b.wrap(&wire);for(unsigned i=0;i<100;++i)b.write(uint8_t(i));
 b.flushSome(32);TEST_ASSERT_EQUAL_UINT(32,wire.tx.size());TEST_ASSERT_EQUAL_UINT(68,b._idx);
 serialDeviceInit(&wire,0);targetPidLooptime=250;currentPidProfile=pidProfilesMutable(0);blackboxConfigMutable()->blackbox_uart=SERIAL_PORT_USART1;blackboxConfigMutable()->blackbox_baud=BAUD_115200;blackboxConfigMutable()->device=BLACKBOX_DEVICE_SERIAL;
 blackboxConfigMutable()->mode=BLACKBOX_MODE_NORMAL;blackboxConfigMutable()->sample_rate=1;
 armingFlags=ARMED;blackboxInit();blackboxUpdate(clockUs);blackboxFinish();
 TEST_ASSERT_TRUE(blackboxServiceShutdown());TEST_ASSERT_FALSE(blackboxShutdownWasTruncated());
 armingFlags=0;serialDeviceInit(nullptr,0);
}
void test_stock_rescue_fixed_altitude_can_descend_then_hold_return_altitude() {
 Control::GpsRescue c;Control::GpsRescueParams p;p.stockCompatibility=true;p.altitudeMode=Control::GPS_RESCUE_ALT_FIXED;
 p.fixedAltitude=10;Control::GpsRescueInput i;i.requested=i.armed=i.homeValid=i.gpsValid=true;
 i.attitudeHealthy=i.headingHealthy=i.altitudeHealthy=i.positionValid=i.velocityValid=true;
 i.sats=8;i.horizontalAccuracy=1;i.north=100;i.altitudeAboveHome=20;i.nowUs=1000000;
 auto o=c.update(i,p);TEST_ASSERT_EQUAL_INT(int(Control::GpsRescuePhase::CLIMB),int(o.phase));TEST_ASSERT_LESS_THAN_FLOAT(0,o.verticalRate);
 TEST_ASSERT_FLOAT_WITHIN(.001f,10,o.targetAltitude);
 i.altitudeAboveHome=10;i.nowUs+=100000;c.update(i,p);i.yawHeading=Control::Navigation::PI_FC;
 i.nowUs+=100000;c.update(i,p);i.altitudeAboveHome=12;i.nowUs+=100000;o=c.update(i,p);
 TEST_ASSERT_LESS_THAN_FLOAT(0,o.verticalRate);TEST_ASSERT_FALSE(o.requestLand);
}
void test_msp_legacy_blackbox_zero_divisors_are_rejected_atomically() {
 Model m;const ModelConfig original=m.config;targetPidLooptime=250;
 TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_BLACKBOX_CONFIG,{3,1,0}).result);
 TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_BLACKBOX_CONFIG,{3,1,1,0,0}).result);
 TEST_ASSERT_EQUAL_MEMORY(&original,&m.config,sizeof(original));
}
void test_output_period_must_contain_the_complete_pwm_pulse() {
 Model m;m.config.output.protocol=ESC_PROTOCOL_PWM;m.config.output.rate=400;
 TEST_ASSERT_TRUE(Utils::configurationValid(m.config));m.config.output.rate=1000;
 TEST_ASSERT_FALSE(Utils::configurationValid(m.config));Output::Mixer mixer(m);
 TEST_ASSERT_EQUAL_INT(0,mixer.begin());TEST_ASSERT_FALSE(m.state.mixer.outputReady);
 m.config.output.rate=400;m.config.output.channel[0].servo=true;m.config.output.servoRate=1000;
 TEST_ASSERT_FALSE(Control::outputTimingValid(m.config));
}
int main(){UNITY_BEGIN();
 RUN_TEST(test_output_period_must_contain_the_complete_pwm_pulse);
 RUN_TEST(test_msp_legacy_blackbox_zero_divisors_are_rejected_atomically);
 RUN_TEST(test_msp_decode_error_does_not_poison_the_next_frame);
 RUN_TEST(test_cli_unsigned_short_preserves_capacity_range);
 RUN_TEST(test_msp_invalid_failsafe_does_not_turn_into_drop);
 RUN_TEST(test_msp_status_api46_remaps_arm_switch_and_has_exact_length);
 RUN_TEST(test_blackbox_close_progresses_without_optional_sample_updates);
 RUN_TEST(test_blackbox_drained_close_and_lifecycle_byte_budget);
 RUN_TEST(test_stock_rescue_fixed_altitude_can_descend_then_hold_return_altitude);
 RUN_TEST(test_record_roundtrip_and_every_byte_corruption_rejected);
 RUN_TEST(test_record_rejects_wrong_layout_and_noncanonical_bool_even_with_valid_crc);
 RUN_TEST(test_record_invalid_config_is_not_encoded);
 RUN_TEST(test_msp_api46_shapes_and_roundtrips);
 RUN_TEST(test_msp_all_truncated_stock_rescue_packets_are_atomic);
 RUN_TEST(test_msp_invalid_composite_battery_write_is_atomic);
 RUN_TEST(test_msp_battery_thresholds_capacity_persist_through_record);
 RUN_TEST(test_serial_policy_rejects_multiple_uart_consumers);
 RUN_TEST(test_mixer_failed_begin_or_attach_never_becomes_ready);
 RUN_TEST(test_mixer_empty_invalid_or_duplicate_outputs_inhibit_arming);
 RUN_TEST(test_stock_rescue_altitude_modes_and_min_sats);
 RUN_TEST(test_vtx_partial_write_preserves_frame_and_crc);
 RUN_TEST(test_vtx_wrong_crc_and_unrelated_ack_do_not_report_ready);
 RUN_TEST(test_vtx_applied_settings_wait_for_matching_ack);
 RUN_TEST(test_vtx_backpressure_and_timeout_are_bounded);
 RUN_TEST(test_vtx_v21_uses_advertised_dbm_power);
 return UNITY_END();}
