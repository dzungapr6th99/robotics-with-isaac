using NLog;

namespace CommonLib
{
    public static class CommonLog
    {
        public static Logger log = LogManager.GetCurrentClassLogger();
        public static Logger logApi = LogManager.GetLogger("logApi");
    }
}
