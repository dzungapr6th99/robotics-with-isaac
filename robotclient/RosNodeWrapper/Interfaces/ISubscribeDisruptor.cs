using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosNodeWrapper.Interfaces
{
    public interface ISubscribeDisruptor<T> where T : class
    {
        public void Enqueue(T item);
    }
}
