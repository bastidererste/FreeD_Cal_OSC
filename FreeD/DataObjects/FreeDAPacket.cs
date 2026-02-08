namespace Track_OSC.FreeD.DataObjects
{
    public class FreeDaPacket : FreeDPacket
    {
        public double XCenter { get; set; }
        public double YCenter { get; set; }

        public double XScale { get; set; }
        public double YScale { get; set; }

        public double LensDistortionA { get; set; }
        public double LensDistortionB { get; set; }

        public double XOffset { get; set; }
        public double YOffset { get; set; }
        public double ZOffset { get; set; }

        public FreeDaPacket()
        {
            this.Type = FreeDPacketType.DA;
        }

        public override string ToString()
        {
            return
                $"FreeDAPacket [{CameraId} {XCenter} {YCenter} {XScale} {YScale} {LensDistortionA} {LensDistortionB} {XOffset} {YOffset} {ZOffset} {Checksum}]";
        }
    }
}