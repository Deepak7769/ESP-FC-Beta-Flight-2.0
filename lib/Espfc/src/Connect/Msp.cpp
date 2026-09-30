#include "Connect/Msp.hpp"
#include "Utils/Crc.hpp"
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace Espfc::Connect {

MspMessage::MspMessage()
    : state(MSP_STATE_IDLE),
      dir(MSP_TYPE_CMD),
      version(MSP_V1),
      flags(0),
      cmd(0),
      expected(0),
      received(0),
      read(0),
      sequence(0),
      checksum(0),
      checksum2(0)
{
  std::memset(buffer, 0, sizeof(buffer));
}

bool MspMessage::isReady() const
{
  return state == MSP_STATE_RECEIVED;
}

bool MspMessage::isCmd() const
{
  return dir == MSP_TYPE_CMD;
}

bool MspMessage::isIdle() const
{
  return state == MSP_STATE_IDLE;
}

int MspMessage::remain() const
{
  return received - read;
}

void MspMessage::advance(size_t size)
{
  const size_t end =
      std::min<size_t>(received, MSP_BUF_SIZE);

  const size_t current =
      std::min<size_t>(read, end);

  read = static_cast<uint16_t>(
      current +
      std::min(size, end - current));
}

uint8_t MspMessage::readU8()
{
  if (read >= received)
  {
    read = received;
    return 0;
  }

  return buffer[read++];
}

uint16_t MspMessage::readU16()
{
  if (remain() < 2)
  {
    read = received;
    return 0;
  }

  uint16_t ret = readU8();

  ret |=
      static_cast<uint16_t>(
          readU8()) << 8;

  return ret;
}

uint32_t MspMessage::readU32()
{
  if (remain() < 4)
  {
    read = received;
    return 0;
  }

  uint32_t ret = readU8();

  ret |=
      static_cast<uint32_t>(
          readU8()) << 8;

  ret |=
      static_cast<uint32_t>(
          readU8()) << 16;

  ret |=
      static_cast<uint32_t>(
          readU8()) << 24;

  return ret;
}

uint16_t MspMessage::append(
    const uint8_t* data,
    size_t len)
{
  if (!data ||
      received >= MSP_BUF_SIZE)
  {
    return received;
  }

  const size_t copyLen =
      std::min<size_t>(
          len,
          MSP_BUF_SIZE - received);

  std::copy_n(
      data,
      copyLen,
      buffer + received);

  received +=
      static_cast<uint16_t>(
          copyLen);

  return received;
}

MspResponse::MspResponse(): len(0) {}

int MspResponse::remain() const
{
  return MSP_BUF_OUT_SIZE - len;
}

void MspResponse::advance(size_t size)
{
  const size_t current =
      std::min<size_t>(len, MSP_BUF_OUT_SIZE);

  len = static_cast<uint16_t>(
      current +
      std::min(size, MSP_BUF_OUT_SIZE - current));
}
void MspResponse::writeData(const char* v, int size)
{
  while (size-- > 0)
  {
    writeU8(*v++);
  }
}

void MspResponse::writeString(const char* v)
{
  writeData(v, std::clamp<int>(std::strlen(v), 0, 168));
}

void MspResponse::writePString(const char* v)
{
  const auto len = std::clamp<int>(std::strlen(v), 0, 168);
  writeU8(len);
  writeData(v, len);
}

void MspResponse::writeU8(uint8_t v)
{
  if (len >= MSP_BUF_OUT_SIZE)
  {
    return;
  }

  data[len++] = v;
}

void MspResponse::writeU16(uint16_t v)
{
  writeU8(v >> 0);
  writeU8(v >> 8);
}

void MspResponse::writeU32(uint32_t v)
{
  writeU8(v >> 0);
  writeU8(v >> 8);
  writeU8(v >> 16);
  writeU8(v >> 24);
}

size_t MspResponse::serialize(uint8_t* buff, size_t len_max) const
{
  switch (version)
  {
    case MSP_V1:
      return serializeV1(buff, len_max);
    case MSP_V2:
      return serializeV2(buff, len_max);
  }
  return 0;
}

size_t MspResponse::serializeV1(uint8_t* buff, size_t len_max) const
{
  // not enough space in target buffer
  if (len + 6ul > len_max) return 0;

  buff[0] = '$';
  buff[1] = 'M';
  buff[2] = result == -1 ? '!' : '>';
  buff[3] = len;
  buff[4] = cmd;

  uint8_t checksum = Utils::crc8_xor(0, &buff[3], 2);
  size_t i = 5;
  for (size_t j = 0; j < len; j++)
  {
    checksum = Utils::crc8_xor(checksum, data[j]);
    buff[i++] = data[j];
  }
  buff[i] = checksum;

  return i + 1;
}

size_t MspResponse::serializeV2(uint8_t* buff, size_t len_max) const
{
  // not enough space in target buffer
  if (len + 9ul > len_max) return 0;

  buff[0] = '$';
  buff[1] = 'X';
  buff[2] = result == -1 ? '!' : '>';
  buff[3] = 0;
  buff[4] = cmd & 0xff;
  buff[5] = (cmd >> 8) & 0xff;
  buff[6] = len & 0xff;
  buff[7] = (len >> 8) & 0xff;

  uint8_t checksum = Utils::crc8_dvb_s2(0, &buff[3], 5);
  size_t i = 8;
  for (size_t j = 0; j < len; j++)
  {
    checksum = Utils::crc8_dvb_s2(checksum, data[j]);
    buff[i++] = data[j];
  }
  buff[i] = checksum;

  return i + 1;
}

} // namespace Espfc::Connect
