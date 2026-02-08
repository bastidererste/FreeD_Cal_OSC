namespace Track_OSC.FreeD.DataObjects
{
    public class FreeD1Packet : FreeDPacket
    {
        public double Pan { get; set; }
        public double Tilt { get; set; }
        public double Roll { get; set; }

        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        public int Zoom { get; set; }
        public int Focus { get; set; }

        public byte[] Spare { get; set; }

        public FreeD1Packet()
        {
            this.Type = FreeDPacketType.D1;
        }

        public override string ToString()
        {
            return $"FreeD1Packet [{CameraId} {Pan} {Tilt} {Roll} {X} {Y} {Z} {Zoom} {Focus} {Spare} {Checksum}]";
        }
    }
}