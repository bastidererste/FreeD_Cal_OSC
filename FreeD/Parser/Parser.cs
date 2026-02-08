using Track_OSC.FreeD.DataObjects;

namespace Track_OSC.FreeD.Parser
{
    public class FreeDParser
    {
        public static FreeDPacket Parse(byte[] data)
        {
            FreeDPacket packet;

            switch (data[0])
            {
                case 0xd1:
                    packet = D1Parser.Parse(data);
                    break;
                
                case 0xda:
                    packet = DAParser.Parse(data);
                    break;
                
                default:
                    return null;
            }

            packet.CameraId = data[1];
            packet.Checksum = data[data.Length - 1];

            return packet;
        }
    }
}