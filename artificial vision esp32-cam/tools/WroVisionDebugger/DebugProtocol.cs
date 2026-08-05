using System;

namespace WroVisionDebugger;

internal enum DebugPacketType : byte
{
    OriginalJpeg = 1,
    CorrectedJpeg = 2,
    Metadata = 3,
    Statistics = 4,
    SystemStatus = 5
}

internal sealed class DebugPacket
{
    public DebugPacket(byte version, DebugPacketType type, uint sequence, byte[] payload)
    {
        Version = version;
        Type = type;
        Sequence = sequence;
        Payload = payload;
    }

    public byte Version { get; }
    public DebugPacketType Type { get; }
    public uint Sequence { get; }
    public byte[] Payload { get; }
}

internal sealed class DebugPacketParser
{
    private const byte MagicFirst = 0xA5;
    private const byte MagicSecond = 0x5A;
    private const int MaximumPayloadLength = 60000;

    private readonly byte[] _payload = new byte[MaximumPayloadLength];
    private ParserState _state = ParserState.WaitMagicFirst;
    private byte _version;
    private DebugPacketType _type;
    private ushort _payloadLength;
    private ushort _payloadIndex;
    private uint _sequence;
    private ushort _receivedCrc;
    private ushort _crc;

    public event Action<DebugPacket>? PacketReceived;

    public long BytesReceived { get; private set; }
    public long PacketsReceived { get; private set; }
    public long CrcErrors { get; private set; }
    public long FormatErrors { get; private set; }

    public void RegisterFormatError()
    {
        FormatErrors++;
    }

    public void Feed(ReadOnlySpan<byte> bytes)
    {
        foreach (byte value in bytes)
        {
            Feed(value);
        }
    }

    private void Feed(byte value)
    {
        BytesReceived++;

        switch (_state)
        {
            case ParserState.WaitMagicFirst:
                if (value == MagicFirst)
                {
                    _state = ParserState.WaitMagicSecond;
                }

                break;

            case ParserState.WaitMagicSecond:
                if (value == MagicSecond)
                {
                    _crc = 0xFFFF;
                    _payloadIndex = 0;
                    _state = ParserState.Version;
                }
                else
                {
                    _state = value == MagicFirst
                        ? ParserState.WaitMagicSecond
                        : ParserState.WaitMagicFirst;
                }

                break;

            case ParserState.Version:
                _version = value;
                UpdateCrc(value);
                _state = ParserState.Type;
                break;

            case ParserState.Type:
                _type = (DebugPacketType)value;
                UpdateCrc(value);
                _state = ParserState.LengthLow;
                break;

            case ParserState.LengthLow:
                _payloadLength = value;
                UpdateCrc(value);
                _state = ParserState.LengthHigh;
                break;

            case ParserState.LengthHigh:
                _payloadLength |= (ushort)(value << 8);
                UpdateCrc(value);
                if (_payloadLength > MaximumPayloadLength)
                {
                    FormatErrors++;
                    Reset();
                }
                else
                {
                    _state = ParserState.Sequence0;
                }

                break;

            case ParserState.Sequence0:
                _sequence = value;
                UpdateCrc(value);
                _state = ParserState.Sequence1;
                break;

            case ParserState.Sequence1:
                _sequence |= (uint)value << 8;
                UpdateCrc(value);
                _state = ParserState.Sequence2;
                break;

            case ParserState.Sequence2:
                _sequence |= (uint)value << 16;
                UpdateCrc(value);
                _state = ParserState.Sequence3;
                break;

            case ParserState.Sequence3:
                _sequence |= (uint)value << 24;
                UpdateCrc(value);
                _state = _payloadLength == 0 ? ParserState.CrcLow : ParserState.Payload;
                break;

            case ParserState.Payload:
                _payload[_payloadIndex++] = value;
                UpdateCrc(value);
                if (_payloadIndex == _payloadLength)
                {
                    _state = ParserState.CrcLow;
                }

                break;

            case ParserState.CrcLow:
                _receivedCrc = value;
                _state = ParserState.CrcHigh;
                break;

            case ParserState.CrcHigh:
                _receivedCrc |= (ushort)(value << 8);
                if (_receivedCrc == _crc)
                {
                    byte[] payload = new byte[_payloadLength];
                    Buffer.BlockCopy(_payload, 0, payload, 0, _payloadLength);
                    PacketsReceived++;
                    PacketReceived?.Invoke(new DebugPacket(_version, _type, _sequence, payload));
                }
                else
                {
                    CrcErrors++;
                }

                Reset();
                break;
        }
    }

    private void UpdateCrc(byte value)
    {
        _crc ^= (ushort)(value << 8);
        for (int bit = 0; bit < 8; bit++)
        {
            _crc = (_crc & 0x8000) != 0
                ? (ushort)((_crc << 1) ^ 0x1021)
                : (ushort)(_crc << 1);
        }
    }

    private void Reset()
    {
        _state = ParserState.WaitMagicFirst;
        _payloadIndex = 0;
        _payloadLength = 0;
        _sequence = 0;
        _receivedCrc = 0;
        _crc = 0;
    }

    private enum ParserState
    {
        WaitMagicFirst,
        WaitMagicSecond,
        Version,
        Type,
        LengthLow,
        LengthHigh,
        Sequence0,
        Sequence1,
        Sequence2,
        Sequence3,
        Payload,
        CrcLow,
        CrcHigh
    }
}

internal sealed class CellMetadata
{
    public byte ColorCode { get; init; }
    public byte Confidence { get; init; }
    public byte Red { get; init; }
    public byte Green { get; init; }
    public byte Blue { get; init; }
    public ushort Hue { get; init; }
    public byte Saturation { get; init; }
    public byte Value { get; init; }
}

internal sealed class GridMetadata
{
    public bool GridValid { get; init; }
    public byte OverallConfidence { get; init; }
    public ushort ProcessingMilliseconds { get; init; }
    public ushort FramesPerSecondTenths { get; init; }
    public CellMetadata[] Cells { get; init; } = Array.Empty<CellMetadata>();
}

internal static class MetadataCodec
{
    private const int CellCount = 12;
    private const int HeaderLength = 6;
    private const int CellLength = 9;
    private const int ExpectedLength = HeaderLength + CellCount * CellLength;

    public static bool TryDecode(ReadOnlySpan<byte> payload, out GridMetadata metadata)
    {
        metadata = new GridMetadata();
        if (payload.Length != ExpectedLength)
        {
            return false;
        }

        CellMetadata[] cells = new CellMetadata[CellCount];
        int offset = HeaderLength;
        for (int index = 0; index < CellCount; index++)
        {
            cells[index] = new CellMetadata
            {
                ColorCode = payload[offset],
                Confidence = payload[offset + 1],
                Red = payload[offset + 2],
                Green = payload[offset + 3],
                Blue = payload[offset + 4],
                Hue = (ushort)(payload[offset + 5] | payload[offset + 6] << 8),
                Saturation = payload[offset + 7],
                Value = payload[offset + 8]
            };
            offset += CellLength;
        }

        metadata = new GridMetadata
        {
            GridValid = payload[0] != 0,
            OverallConfidence = payload[1],
            ProcessingMilliseconds = (ushort)(payload[2] | payload[3] << 8),
            FramesPerSecondTenths = (ushort)(payload[4] | payload[5] << 8),
            Cells = cells
        };
        return true;
    }

    public static string ColorName(byte colorCode)
    {
        return colorCode switch
        {
            1 => "Amarillo",
            2 => "Azul",
            3 => "Verde",
            4 => "Blanco",
            _ => "Desconocido"
        };
    }
}
