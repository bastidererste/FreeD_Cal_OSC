using Track_OSC.FreeD.DataObjects;
using Track_OSC.FreeD.Util;

namespace Track_OSC.FreeD.Parser
{
    public static class DAParser
    {
        public static FreeDaPacket Parse(byte[] data)
        {
            var packet = new FreeDaPacket();

            packet.XCenter = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 2, 3));
            packet.YCenter = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 5, 3));
            
            packet.XScale = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 8, 3));
            packet.YScale = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 11, 3));
            
            packet.LensDistortionA = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 14, 3));
            packet.LensDistortionB = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 17, 3));
            
            packet.XOffset = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 20, 3));
            packet.YOffset = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 23, 3));
            packet.ZOffset = FreeDUtils.ParseDouble24(FreeDUtils.SubArray(data, 26, 3));

            return packet;
        }
    }
}