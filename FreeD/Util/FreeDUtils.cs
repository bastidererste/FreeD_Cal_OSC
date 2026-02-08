using System;

namespace Track_OSC.FreeD.Util
{
    public class FreeDUtils
    {
        public static double ParseDouble24(byte[] data)
        {

            
            
            int r = 1;

            r <<= 8;
            r |= data[0];
            r <<= 8;
            r |= data[1];
            r <<= 8;
            r |= data[2];
            bool bit_is_on = (((int)r) & 0x00800000) != 0;
            if (!bit_is_on)
                r &= 0x00ffffff;

            return (double)r / 64.0;
        }

        public static int ParseInt24(byte[] data)
        {



            int r = 1;

            r <<= 8;
            r |= data[0];
            r <<= 8;
            r |= data[1];
            r <<= 8;
            r |= data[2];
            bool bit_is_on = (((int)r) & 0x00800000) != 0;
            if (!bit_is_on)
                r &= 0x00ffffff;

            return r;
        }



        public static double ParseComplement8(byte[] data)
        {
            int r = -1;

            r <<= 8;
            r |= data[0];
            r <<= 8;
            r |= data[1];
            r <<= 8;
            r |= data[2];
            bool bit_is_on = (((int)r) & 0x00800000) != 0;
            if (!bit_is_on)
                r &= 0x00ffffff;

            return (double)r / 32768.0;
            //return ParseInt24(data) / 32768;
        }

        public static double ParseComplement17(byte[] data)
        {
            return ParseDouble24(data) / 64;
        }

        public static T[] SubArray<T>(T[] data, int index, int length)
        {
            T[] result = new T[length];
            Array.Copy(data, index, result, 0, length);
            return result;
        }
    }
}