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
#include "Sensor/GyroSensor.hpp"
#include "Device/InputPPM.h"
#include "Device/InputIBUS.hpp"
#include "ResourceManager.h"
#include "Control/Controller.h"
#include "msp/msp_protocol.h"
#include "Utils/Crc.hpp"
#include <ArduinoFake.h>
#include <unity.h>
#include <vector>
#include <deque>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <sstream>
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
 auto version=process(m,MSP_FC_VERSION);
 const char* text="2026.6.1";
 TEST_ASSERT_EQUAL_UINT(4+strlen(text),version.len);
 TEST_ASSERT_EQUAL_UINT8(26,version.data[0]);TEST_ASSERT_EQUAL_UINT8(6,version.data[1]);
 TEST_ASSERT_EQUAL_UINT8(1,version.data[2]);TEST_ASSERT_EQUAL_UINT8(strlen(text),version.data[3]);
 TEST_ASSERT_EQUAL_MEMORY(text,version.data+4,strlen(text));
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

static void executeCli(Connect::Cli& cli, CliCmd& cmd,
                       Espfc::Stream::Printer& printer, const char* text)
{
  for (; *text; ++text) cli.process(*text, cmd, printer);
}

void test_cli_output_change_requires_reboot_after_cancel()
{
  Model model;
  model.config.output.protocol = ESC_PROTOCOL_PWM;
  model.config.output.rate = 400;
  Output::Mixer mixer(model);
  TEST_ASSERT_EQUAL_INT(1, mixer.begin());
  TestWire stream;
  Espfc::Stream::Printer printer(stream);
  Connect::Cli cli(model);
  CliCmd cmd{};
  cli.process('#', cmd, printer);
  executeCli(cli, cmd, printer, "set pin_output_0 8\n");
  TEST_ASSERT_EQUAL_INT(8, model.config.pin[PIN_OUTPUT_0]);
  TEST_ASSERT_FALSE(model.state.mixer.outputReady);
  TEST_ASSERT_TRUE(model.getRebootRequired());
  cli.process(0x04, cmd, printer);
  TEST_ASSERT_FALSE(model.state.mode.armingDisabledFlags & ARMING_DISABLED_CLI);
  TEST_ASSERT_TRUE(model.armingDisabled());
  TEST_ASSERT_TRUE(model.state.mode.armingDisabledFlags & ARMING_DISABLED_REBOOT_REQUIRED);
}

void test_cli_invalid_idle_and_unknown_parameter_preserve_readiness()
{
  Model model;
  model.config.output.protocol = ESC_PROTOCOL_PWM;
  model.config.output.rate = 400;
  Output::Mixer mixer(model);
  TEST_ASSERT_EQUAL_INT(1, mixer.begin());
  const ModelConfig before = model.config;
  TestWire stream;
  Espfc::Stream::Printer printer(stream);
  Connect::Cli cli(model);
  CliCmd cmd{};
  cli.process('#', cmd, printer);
  for (const char* command : {"set output_motor_idle -1\n",
                              "set output_motor_idle 2001\n",
                              "set missing_setting 1\n"})
  {
    executeCli(cli, cmd, printer, command);
    TEST_ASSERT_EQUAL_MEMORY(&before, &model.config, sizeof(before));
    TEST_ASSERT_TRUE(model.state.mixer.outputReady);
    TEST_ASSERT_FALSE(model.getRebootRequired());
  }
  cli.process(0x04, cmd, printer);
  TEST_ASSERT_FALSE(model.armingDisabled());
}

void test_cli_noninteractive_configuration_cannot_bypass_reboot_gate()
{
  Model model;
  model.config.output.protocol = ESC_PROTOCOL_PWM;
  model.config.output.rate = 400;
  Output::Mixer mixer(model);
  TEST_ASSERT_EQUAL_INT(1, mixer.begin());
  TestWire stream;
  Espfc::Stream::Printer printer(stream);
  Connect::Cli cli(model);
  CliCmd cmd{};
  cli.process(0x02, cmd, printer);
  executeCli(cli, cmd, printer, "set output_motor_rate 450\n");
  cli.process(0x03, cmd, printer);
  TEST_ASSERT_EQUAL_INT(450, model.config.output.rate);
  TEST_ASSERT_FALSE(model.state.mixer.outputReady);
  TEST_ASSERT_TRUE(model.getRebootRequired());
  TEST_ASSERT_TRUE(model.armingDisabled());
}

void test_cli_other_configuration_mutators_invalidate_readiness()
{
  for (const char* command : {"defaults\n", "preset scaler\n", "preset modes\n", "load\n"})
  {
    Model model;
    model.config.output.protocol = ESC_PROTOCOL_PWM;
    model.config.output.rate = 400;
    Output::Mixer mixer(model);
    TEST_ASSERT_EQUAL_INT(1, mixer.begin());
    TestWire stream;
    Espfc::Stream::Printer printer(stream);
    Connect::Cli cli(model);
    CliCmd cmd{};
    executeCli(cli, cmd, printer, command);
    TEST_ASSERT_FALSE_MESSAGE(model.state.mixer.outputReady, command);
    TEST_ASSERT_TRUE_MESSAGE(model.getRebootRequired(), command);
    TEST_ASSERT_TRUE_MESSAGE(model.armingDisabled(), command);
  }
}

void test_msp_invalid_motor_idle_is_rejected_before_narrowing()
{
  for (const uint16_t idle : {2001u, 32767u, 32768u, 65535u})
  {
    Model model;
    const ModelConfig before = model.config;
    const auto reply = process(model, MSP_SET_ADVANCED_CONFIG,
        {0, 1, 1, ESC_PROTOCOL_PWM, 0x90, 0x01,
         uint8_t(idle), uint8_t(idle >> 8)});
    TEST_ASSERT_EQUAL_INT(-1, reply.result);
    TEST_ASSERT_EQUAL_MEMORY(&before, &model.config, sizeof(before));
    TEST_ASSERT_FALSE(model.getRebootRequired());
  }
}

void test_msp_valid_motor_idle_boundaries_are_not_silently_changed()
{
  for (const uint16_t idle : {0u, 550u, 2000u})
  {
    Model model;
    const auto reply = process(model, MSP_SET_ADVANCED_CONFIG,
        {0, 1, 1, ESC_PROTOCOL_PWM, 0x90, 0x01,
         uint8_t(idle), uint8_t(idle >> 8)});
    TEST_ASSERT_EQUAL_INT(1, reply.result);
    TEST_ASSERT_EQUAL_INT(idle, model.config.output.motorIdle);
  }
}

void test_configuration_record_rejects_invalid_idle_and_throttle_span()
{
  ModelConfig config;
  uint8_t record[2048];
  for (const int idle : {-1, 2001})
  {
    config.output.motorIdle = idle;
    TEST_ASSERT_FALSE(Utils::configurationValid(config));
    TEST_ASSERT_FALSE(Utils::ConfigRecord::encode(config, record, sizeof(record)));
  }
  config.output.minCommand = 1950;
  config.output.maxThrottle = 2000;
  config.output.motorIdle = 500;
  TEST_ASSERT_TRUE(Utils::configurationValid(config));
  config.output.motorIdle = 501;
  TEST_ASSERT_FALSE(Utils::configurationValid(config));
  TEST_ASSERT_FALSE(Utils::ConfigRecord::encode(config, record, sizeof(record)));
}

void test_msp_motor_range_cannot_silently_reduce_existing_idle()
{
  Model model;
  const ModelConfig before = model.config;
  const std::vector<uint8_t> narrowRange = {0, 0, 0xd0, 0x07, 0x9e, 0x07};
  TEST_ASSERT_EQUAL_INT(-1, process(model, MSP_SET_MOTOR_CONFIG, narrowRange).result);
  TEST_ASSERT_EQUAL_MEMORY(&before, &model.config, sizeof(before));
  TEST_ASSERT_EQUAL_INT(1, process(model, MSP_SET_ADVANCED_CONFIG,
      {0, 1, 1, ESC_PROTOCOL_PWM, 0x90, 0x01, 0xf4, 0x01}).result);
  TEST_ASSERT_EQUAL_INT(1, process(model, MSP_SET_MOTOR_CONFIG, narrowRange).result);
  TEST_ASSERT_EQUAL_INT(1950, model.config.output.minCommand);
  TEST_ASSERT_EQUAL_INT(500, model.config.output.motorIdle);
}


class ClosureMockSpiBus : public Device::BusDevice
{
public:
  BusType busType{BUS_SPI};
  BusType getType() const override { return busType; }
  int8_t read(uint8_t, uint8_t, uint8_t, uint8_t*) override { return 0; }
  int8_t readFast(uint8_t, uint8_t, uint8_t, uint8_t*) override { return 0; }
  bool write(uint8_t, uint8_t, uint8_t, const uint8_t*) override { return true; }
};

class ClosureMockGyro : public Device::GyroDevice
{
public:
  bool connected{true};
  bool configured{true};
  bool readingsWork{true};
  int samples{0};
  int checks{0};
  int begin(Device::BusDevice* bus) override { setBus(bus, 0); return 1; }
  int begin(Device::BusDevice* bus, uint8_t address) override { setBus(bus, address); return 1; }
  DeviceType getType() const override { return GYRO_MPU6500; }
  int readGyro(VectorInt16& v) override { ++samples; v = VectorInt16(0, 0, 0); return readingsWork ? 1 : 0; }
  bool configurationValid() const override { return configured; }
  int readAccel(VectorInt16& v) override { v = VectorInt16(0, 0, 0); return 1; }
  void setDLPFMode(uint8_t) override {}
  int getRate() const override { return 4000; }
  void setRate(int) override {}
  bool testConnection() override { ++checks; return connected; }
};

void test_gyro_spi_identity_fault_latches_until_disarmed_begin()
{
  Model m;
  m.logger.begin();
  m.config.gyro.dev = GYRO_MPU6500;
  m.state.loopTimer.rate = 4000;
  m.state.loopTimer.interval = 250;
  m.state.gyro.timer.rate = 4000;
  m.state.gyro.timer.interval = 250;
  m.state.gyro.present = true;
  ClosureMockSpiBus bus;
  ClosureMockGyro device;
  device.setBus(&bus, 0x68);
  m.state.gyro.dev = &device;
  Sensor::GyroSensor gyro(m);
  device.connected = false;
  TEST_ASSERT_EQUAL_INT(0, gyro.begin());
  TEST_ASSERT_FALSE(m.state.gyro.sampleValid);
  const int count = device.samples;
  TEST_ASSERT_EQUAL_INT(0, gyro.read());
  TEST_ASSERT_EQUAL_INT(count, device.samples);
  device.connected = true;
  device.configured = false;
  TEST_ASSERT_EQUAL_INT(0, gyro.begin());
  // Even if another subsystem incorrectly marks the sensor present, a
  // failed configuration may not unlatch its identity failure.
  m.state.gyro.present = true;
  TEST_ASSERT_EQUAL_INT(0, gyro.read());
  device.configured = true;
  m.updateModes(1u << MODE_ARMED);
  TEST_ASSERT_EQUAL_INT(0, gyro.begin());
  TEST_ASSERT_EQUAL_INT(0, gyro.read());
  m.updateModes(0);
  TEST_ASSERT_EQUAL_INT(1, gyro.begin());
  TEST_ASSERT_EQUAL_INT(1, gyro.read());
  TEST_ASSERT_TRUE(m.state.gyro.sampleValid);
  TEST_ASSERT_EQUAL_INT(count + 1, device.samples);
  device.readingsWork = false;
  TEST_ASSERT_EQUAL_INT(0, gyro.read());
  TEST_ASSERT_FALSE(m.state.gyro.sampleValid);
  device.readingsWork = true;
  TEST_ASSERT_EQUAL_INT(1, gyro.read());
  // Periodic SPI WHO_AM_I failure must also lock out subsequent zero reads.
  clockUs += 100000;
  device.connected = false;
  TEST_ASSERT_EQUAL_INT(0, gyro.read());
  TEST_ASSERT_FALSE(m.state.gyro.sampleValid);
  device.connected = true;
  TEST_ASSERT_EQUAL_INT(0, gyro.read());
  TEST_ASSERT_EQUAL_INT(1, gyro.begin());
  TEST_ASSERT_EQUAL_INT(1, gyro.read());
}

void test_ppm_publishes_complete_immutable_receiver_snapshots()
{
  Device::InputPPM ppm;
  ppm.begin(-1);
  auto pulse = [&](uint32_t us) { clockUs += us; ppm.handle(); };
  pulse(4000);
  for (int frame = 0; frame < 3; ++frame)
  {
    for (uint32_t us : {1300u + 100u * uint32_t(frame), 1400u, 1100u, 1600u}) pulse(us);
    pulse(4000);
  }
  TEST_ASSERT_EQUAL_INT(INPUT_RECEIVED, ppm.update());
  TEST_ASSERT_EQUAL_UINT(4, ppm.getChannelCount());
  TEST_ASSERT_EQUAL_UINT16(1500, ppm.get(0));
  TEST_ASSERT_EQUAL_UINT16(1100, ppm.get(2));
  pulse(1700); pulse(1200);
  TEST_ASSERT_EQUAL_INT(INPUT_IDLE, ppm.update());
  TEST_ASSERT_EQUAL_UINT16(1500, ppm.get(0));
  TEST_ASSERT_EQUAL_UINT16(1400, ppm.get(1));
  pulse(1050); pulse(1800); pulse(4000);
  TEST_ASSERT_EQUAL_INT(INPUT_RECEIVED, ppm.update());
  TEST_ASSERT_EQUAL_UINT16(1700, ppm.get(0));
  TEST_ASSERT_EQUAL_UINT16(1200, ppm.get(1));
  TEST_ASSERT_EQUAL_UINT16(1050, ppm.get(2));
  uint16_t batch[4]{};
  ppm.get(batch, 4);
  TEST_ASSERT_EQUAL_UINT16(1800, batch[3]);
}

void test_rate_type_and_msp_rate_limits_reject_invalid_atomic_writes()
{
  ModelConfig config;
  config.input.rateType = 5;
  TEST_ASSERT_FALSE(Utils::configurationValid(config));
  config.input.rateType = 3;
  for (int channel = 0; channel < 3; ++channel)
  {
    for (int bad : {0, 1999, 32767})
    {
      config.input.rateLimit[channel] = bad;
      TEST_ASSERT_FALSE(Utils::configurationValid(config));
      config.input.rateLimit[channel] = 1998;
    }
  }
  Model model;
  const ModelConfig before = model.config;
  const auto reply = process(model, MSP_RC_TUNING);
  TEST_ASSERT_EQUAL_INT(1, reply.result);
  TEST_ASSERT_EQUAL_UINT(23, reply.len);
  const std::vector<uint8_t> baseline(reply.data, reply.data + reply.len);
  for (int offset : {16, 18, 20})
  {
    auto request = baseline;
    request[offset] = 0xff;
    request[offset + 1] = 0xff;
    TEST_ASSERT_EQUAL_INT(-1, process(model, MSP_SET_RC_TUNING, request).result);
    TEST_ASSERT_EQUAL_MEMORY(&before, &model.config, sizeof(before));
  }
  auto badAlgorithm = baseline;
  badAlgorithm[22] = 5;
  TEST_ASSERT_EQUAL_INT(-1, process(model, MSP_SET_RC_TUNING, badAlgorithm).result);
  TEST_ASSERT_EQUAL_MEMORY(&before, &model.config, sizeof(before));
}

void test_mixer_rule_validator_checks_all_rules_and_terminator()
{
  auto builtin = Output::Mixers::getMixer(FC_MIXER_QUADX, MixerConfig{});
  TEST_ASSERT_TRUE(mixerConfigValid(builtin, 4));
  MixerEntry rules[MIXER_RULE_MAX]{};
  rules[0] = MixerEntry(MIXER_SOURCE_ROLL, 0, 100);
  TEST_ASSERT_TRUE(mixerConfigValid(MixerConfig(4, rules), 4));
  rules[0] = MixerEntry(MIXER_SOURCE_ROLL, 4, 100);
  TEST_ASSERT_FALSE(mixerConfigValid(MixerConfig(4, rules), 4));
  rules[0] = MixerEntry(MIXER_SOURCE_ROLL, 0, 1001);
  TEST_ASSERT_FALSE(mixerConfigValid(MixerConfig(4, rules), 4));
  rules[0] = MixerEntry(MIXER_SOURCE_ROLL, 0, 100);
  TEST_ASSERT_FALSE(mixerConfigValid(MixerConfig(5, rules), 4));
  for (size_t i = 0; i < MIXER_RULE_MAX; ++i)
    rules[i] = MixerEntry(MIXER_SOURCE_ROLL, 0, 100);
  TEST_ASSERT_FALSE(mixerConfigValid(MixerConfig(4, rules), 4));
  ModelConfig config;
  config.mixer.type = FC_MIXER_CUSTOM;
  config.customMixerCount = 4;
  config.customMixes[0] = MixerEntry(MIXER_SOURCE_ROLL, 4, 100);
  TEST_ASSERT_FALSE(Utils::configurationValid(config));
  config.customMixes[0] = MixerEntry(MIXER_SOURCE_ROLL, 0, 100);
  TEST_ASSERT_TRUE(Utils::configurationValid(config));
}

namespace Espfc {
class ResourceManagerTestAccess {
public:
 static void apply(ResourceManager& manager, float load, uint32_t now) { manager.applyPolicy(load, now); }
};
}
void test_disarmed_msp_survives_overload_peak_and_deadline_stress() {
 Model m;ResourceManager manager(m);manager.begin();
 ResourceManagerTestAccess::apply(manager,82,clockUs);
 TEST_ASSERT_TRUE(manager.mspAllowed());TEST_ASSERT_FALSE(manager.blackboxAllowed());
 m.updateModes(1u<<MODE_ARMED);TEST_ASSERT_FALSE(manager.mspAllowed());
 m.updateModes(0);TEST_ASSERT_TRUE(manager.mspAllowed());
 manager.noteFlightDeadlineMisses(2);clockUs+=10000;
 ResourceManagerTestAccess::apply(manager,73,clockUs);
 TEST_ASSERT_TRUE(manager.mspAllowed());
 m.updateModes(1u<<MODE_ARMED);TEST_ASSERT_FALSE(manager.mspAllowed());
 clockUs+=1100000;ResourceManagerTestAccess::apply(manager,30,clockUs);
 TEST_ASSERT_TRUE(manager.mspAllowed());
}
void test_i2c_bus_clock_budget_and_spi_rate_are_independent() {
 Model m;ClosureMockSpiBus bus;ClosureMockGyro gyro;gyro.setBus(&bus,0x68);
 m.state.gyro.dev=&gyro;m.state.gyro.clock=8000;m.config.loopSync=1;
 for (int speed : {400,100,10}) {
  bus.busType=BUS_I2C;m.config.i2cSpeed=speed;m.begin();
  TEST_ASSERT_LESS_OR_EQUAL_UINT(speed*1000/400,m.state.gyro.timer.rate);
  TEST_ASSERT_EQUAL_UINT(m.state.gyro.timer.rate,m.state.loopTimer.rate);
  TEST_ASSERT_LESS_OR_EQUAL_UINT(m.state.gyro.timer.rate,m.state.accel.timer.rate);
 }
 bus.busType=BUS_SPI;m.begin();
 TEST_ASSERT_EQUAL_UINT(ESPFC_GYRO_SPI_RATE_MAX,m.state.gyro.timer.rate);
}
void test_mode_extra_reports_every_condition_including_last_links() {
 Model m;m.config.conditions[7].id=MODE_ANGLE;m.config.conditions[7].logicMode=1;
 m.config.conditions[7].linkId=MODE_AIRMODE;
 auto r=process(m,MSP_MODE_RANGES_EXTRA);
 TEST_ASSERT_EQUAL_UINT(1+3*ACTUATOR_CONDITIONS,r.len);
 TEST_ASSERT_EQUAL_UINT8(ACTUATOR_CONDITIONS,r.data[0]);
 TEST_ASSERT_EQUAL_UINT8(1,r.data[1+7*3+1]);
 TEST_ASSERT_EQUAL_UINT8(Connect::MspBoxMapping::encode(MODE_AIRMODE),r.data[1+7*3+2]);
}
void test_pid_ten_slot_packets_update_runtime_and_preserve_stored_indices() {
 Model m;m.state.gyro.clock=1000;m.config.loopSync=1;m.begin();
 Control::Controller controller(m);controller.begin();
 m.setConfigChangeListener([&](ModelChangeEvent event){controller.reload(event);});
 auto r=process(m,MSP_PID);TEST_ASSERT_EQUAL_UINT(30,r.len);
 TEST_ASSERT_EQUAL_UINT8(m.config.pid[FC_PID_LEVEL].P,r.data[3*3]);
#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
 TEST_ASSERT_EQUAL_UINT8(m.config.pid[FC_PID_ALT].P,r.data[FC_PID_ALT*3]);
 TEST_ASSERT_EQUAL_UINT8(m.config.pid[FC_PID_VEL].P,r.data[FC_PID_VEL*3]);
#else
 TEST_ASSERT_EQUAL_UINT8(0,r.data[FC_PID_ALT*3]);
#endif
 std::vector<uint8_t> p(r.data,r.data+r.len);p[0]=37;p[3*3]=42;
 const float before=m.state.innerPid[0].Kp;
 TEST_ASSERT_EQUAL_INT(1,process(m,MSP_SET_PID,p).result);
 TEST_ASSERT_EQUAL_UINT8(42,m.config.pid[FC_PID_LEVEL].P);
 TEST_ASSERT_FLOAT_WITHIN(0.00001f,before*37.0f/45.0f,m.state.innerPid[0].Kp);
 const ModelConfig stored=m.config;p[FC_PID_MAG*3]=1;
 TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_PID,p).result);
 TEST_ASSERT_EQUAL_MEMORY(&stored,&m.config,sizeof(stored));
 TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_PID,std::vector<uint8_t>(16,0)).result);
 p[FC_PID_MAG*3]=0;p.resize(15);
 TEST_ASSERT_EQUAL_INT(1,process(m,MSP_SET_PID,p).result);
}
void test_api46_feedforward_cutoff_reloads_real_filter_and_provider_requires_reboot() {
 Model m;m.state.gyro.clock=1000;m.config.loopSync=1;m.config.input.filterThrottle.freq=90;m.begin();
 Control::Controller controller(m);controller.begin();
 m.setConfigChangeListener([&](ModelChangeEvent event){controller.reload(event);});
 auto r=process(m,MSP_RX_CONFIG);std::vector<uint8_t> p(r.data,r.data+r.len);
 p[0]=SERIALRX_IBUS;p[26]=25;
 TEST_ASSERT_EQUAL_INT(1,process(m,MSP_SET_RX_CONFIG,p).result);
 TEST_ASSERT_EQUAL_UINT16(25,m.config.input.filterDerivative.freq);
 TEST_ASSERT_EQUAL_UINT16(90,m.config.input.filterThrottle.freq);
 TEST_ASSERT_EQUAL_UINT16(25,m.state.innerPid[0].ftermFilter._conf.freq);
 TEST_ASSERT_TRUE(m.getRebootRequired());
 const ModelConfig before=m.config;m.updateModes(1u<<MODE_ARMED);p[26]=30;
 TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_RX_CONFIG,p).result);
 TEST_ASSERT_EQUAL_MEMORY(&before,&m.config,sizeof(before));
}
void test_ibus_decodes_real_frame_and_rejects_corruption() {
 TestWire wire;Device::InputIBUS ibus;ibus.begin(&wire);
 std::vector<uint8_t> frame={32,0x40};uint16_t checksum=0xffff-32-0x40;
 for (unsigned i=0;i<14;++i) {
  unsigned value=1000+i*50;frame.push_back(value&255);frame.push_back(value>>8);
  checksum-=(value&255)+(value>>8);
 }
 frame.push_back(checksum&255);frame.push_back(checksum>>8);
 for (uint8_t b:frame)wire.rx.push_back(b);
 TEST_ASSERT_EQUAL_INT(INPUT_RECEIVED,ibus.update());
 for(unsigned i=0;i<14;++i)TEST_ASSERT_EQUAL_UINT16(1000+i*50,ibus.get(i));
 clockUs+=1000;frame[8]^=1;for(uint8_t b:frame)wire.rx.push_back(b);
 TEST_ASSERT_EQUAL_INT(INPUT_IDLE,ibus.update());
 TEST_ASSERT_EQUAL_UINT16(1150,ibus.get(3));
}
void test_api46_sensor_alignment_preserves_device_and_rejects_second_gyro() {
 Model m;auto r=process(m,MSP_SENSOR_ALIGNMENT);
 TEST_ASSERT_EQUAL_UINT(7,r.len);TEST_ASSERT_EQUAL_UINT8(0,r.data[4]);
 const auto dev=m.config.gyro.dev;
 std::vector<uint8_t> p={0,0,0,0,2,0};
 TEST_ASSERT_EQUAL_INT(1,process(m,MSP_SET_SENSOR_ALIGNMENT,p).result);
 TEST_ASSERT_EQUAL_INT(dev,m.config.gyro.dev);TEST_ASSERT_EQUAL_UINT8(2,m.config.gyro.align);
 TEST_ASSERT_TRUE(m.getRebootRequired());const ModelConfig before=m.config;
 p[3]=1;TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_SENSOR_ALIGNMENT,p).result);
 TEST_ASSERT_EQUAL_MEMORY(&before,&m.config,sizeof(before));
 p[3]=0;p[5]=1;TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_SENSOR_ALIGNMENT,p).result);
 TEST_ASSERT_EQUAL_MEMORY(&before,&m.config,sizeof(before));
 p.resize(5);TEST_ASSERT_EQUAL_INT(-1,process(m,MSP_SET_SENSOR_ALIGNMENT,p).result);
}
// Optional export is consumed by the exact Configurator decoder/serializer
// regression. Values come from real handlers and state, never a copied MSP model.
void test_configurator_packet_export_and_client_write_readback() {
 Model m;m.state.currentMixer=Output::Mixers::getMixer(FC_MIXER_QUADX,MixerConfig{});
 m.config.featureMask=Espfc::FEATURE_RX_SERIAL|Espfc::FEATURE_TELEMETRY;
 m.config.input.serialRxProvider=SERIALRX_IBUS;m.state.input.channelCount=14;
 for(unsigned i=0;i<14;++i)m.state.input.us[i]=1000+i*50;
 if(const char* input=std::getenv("ESPFC_MSP_WRITES")) {
  std::ifstream writes(input);TEST_ASSERT_TRUE(writes.good());
  unsigned command,length,value;while(writes>>command>>length) {
   std::vector<uint8_t> payload;
   for(unsigned i=0;i<length;++i){writes>>value;payload.push_back(value);}
   auto response=process(m,command,payload);
   char context[80];snprintf(context,sizeof(context),"Real Configurator command %u length %u",command,length);
   TEST_ASSERT_EQUAL_INT_MESSAGE(1,response.result,context);
  }
  uint8_t record[2048];ModelConfig restored;
  TEST_ASSERT_TRUE(Utils::ConfigRecord::encode(m.config,record,sizeof(record)));
  TEST_ASSERT_TRUE(Utils::ConfigRecord::decode(record,sizeof(record),restored));
  m.config=restored; // readback after the production record codec
 }
 if(const char* output=std::getenv("ESPFC_MSP_FIXTURES")) {
  std::ofstream out(output);TEST_ASSERT_TRUE(out.good());out<<"[";bool first=true;
  for(uint16_t command : {MSP_API_VERSION,MSP_FC_VARIANT,MSP_FC_VERSION,MSP_BOARD_INFO,
   MSP_BUILD_INFO,MSP_STATUS_EX,MSP_FEATURE_CONFIG,MSP_BEEPER_CONFIG,MSP_MIXER_CONFIG,
   MSP_MOTOR_CONFIG,MSP_MOTOR,MSP_MOTOR_TELEMETRY,MSP_MOTOR_3D_CONFIG,MSP_ADVANCED_CONFIG,
   MSP_RX_CONFIG,MSP_RX_MAP,MSP_RC,MSP_RC_DEADBAND,MSP_RSSI_CONFIG,MSP_RC_TUNING,
   MSP_PID,MSP_PID_ADVANCED,MSP_PIDNAMES,MSP_FILTER_CONFIG,MSP_SIMPLIFIED_TUNING,
   MSP_BATTERY_CONFIG,MSP_BATTERY_STATE,MSP_FAILSAFE_CONFIG,MSP_RXFAIL_CONFIG,
   MSP_MODE_RANGES,MSP_MODE_RANGES_EXTRA,MSP_BOXNAMES,MSP_BOXIDS,MSP_SENSOR_CONFIG,
   MSP_SENSOR_ALIGNMENT,MSP_BOARD_ALIGNMENT_CONFIG,MSP_ARMING_CONFIG,MSP_BLACKBOX_CONFIG,
   MSP_DATAFLASH_SUMMARY,MSP_CF_SERIAL_CONFIG,MSP_GPS_CONFIG}) {
   auto r=process(m,command);TEST_ASSERT_EQUAL_INT(1,r.result);
   if(!first)out<<",";first=false;out<<"{\"code\":"<<command<<",\"payload\":[";
   for(unsigned i=0;i<r.len;++i){if(i)out<<",";out<<unsigned(r.data[i]);}out<<"]}";
  }
  out<<"]";
 }
}

int main(){UNITY_BEGIN();
 RUN_TEST(test_disarmed_msp_survives_overload_peak_and_deadline_stress);
 RUN_TEST(test_i2c_bus_clock_budget_and_spi_rate_are_independent);
 RUN_TEST(test_mode_extra_reports_every_condition_including_last_links);
 RUN_TEST(test_pid_ten_slot_packets_update_runtime_and_preserve_stored_indices);
 RUN_TEST(test_api46_feedforward_cutoff_reloads_real_filter_and_provider_requires_reboot);
 RUN_TEST(test_ibus_decodes_real_frame_and_rejects_corruption);
 RUN_TEST(test_api46_sensor_alignment_preserves_device_and_rejects_second_gyro);
 RUN_TEST(test_configurator_packet_export_and_client_write_readback);
 RUN_TEST(test_gyro_spi_identity_fault_latches_until_disarmed_begin);
 RUN_TEST(test_ppm_publishes_complete_immutable_receiver_snapshots);
 RUN_TEST(test_rate_type_and_msp_rate_limits_reject_invalid_atomic_writes);
 RUN_TEST(test_mixer_rule_validator_checks_all_rules_and_terminator);
 RUN_TEST(test_cli_output_change_requires_reboot_after_cancel);
 RUN_TEST(test_cli_invalid_idle_and_unknown_parameter_preserve_readiness);
 RUN_TEST(test_cli_noninteractive_configuration_cannot_bypass_reboot_gate);
 RUN_TEST(test_cli_other_configuration_mutators_invalidate_readiness);
 RUN_TEST(test_msp_invalid_motor_idle_is_rejected_before_narrowing);
 RUN_TEST(test_msp_valid_motor_idle_boundaries_are_not_silently_changed);
 RUN_TEST(test_configuration_record_rejects_invalid_idle_and_throttle_span);
 RUN_TEST(test_msp_motor_range_cannot_silently_reduce_existing_idle);
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
