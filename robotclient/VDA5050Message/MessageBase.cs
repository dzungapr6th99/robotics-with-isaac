using VDA5050Message.Base;

namespace VDA5050Message
{
    public class MessageBase : VDA5050MessageBase
    {
        public Header Header;

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }
}
