using Track_OSC.FreeD.DataObjects;
using Track_OSC.FreeD.Util;

namespace Track_OSC.FreeD.Parser
{
    public static class D1Parser
    {
        public static FreeD1Packet Parse(byte[] data)
        {
            var packet = new FreeD1Packet();

            packet.Pan = FreeDUtils.ParseComplement8(FreeDUtils.SubArray(data, 2, 3));
            packet.Tilt = FreeDUtils.ParseComplement8(FreeDUtils.SubArray(data, 5, 3));
            packet.Roll = FreeDUtils.ParseComplement8(FreeDUtils.SubArray(data, 8, 3));

            packet.X = FreeDUtils.ParseComplement17(FreeDUtils.SubArray(data, 11, 3));
            packet.Y = FreeDUtils.ParseComplement17(FreeDUtils.SubArray(data, 14, 3));
            packet.Z = FreeDUtils.ParseComplement17(FreeDUtils.SubArray(data, 17, 3));

            packet.Zoom = FreeDUtils.ParseInt24(FreeDUtils.SubArray(data, 20, 3));
            packet.Focus = FreeDUtils.ParseInt24(FreeDUtils.SubArray(data, 23, 3));


            packet.Spare = FreeDUtils.SubArray(data, 26, 2);

            return packet;
        }
    }
}