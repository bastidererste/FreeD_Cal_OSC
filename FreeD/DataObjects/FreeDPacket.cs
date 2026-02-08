namespace Track_OSC.FreeD.DataObjects
{
    public abstract class FreeDPacket
    {
        // ReSharper disable once UnusedAutoPropertyAccessor.Global
        public FreeDPacketType Type { get; set; }

        public byte CameraId { get; set; }

        public byte Checksum { get; set; }

        public override string ToString()
        {
            return $"FreeDPacket [{CameraId} {Checksum}]";
        }
    }

    public enum FreeDPacketType
    {
        D1,
        // ReSharper disable once InconsistentNaming
        DA
    }
}