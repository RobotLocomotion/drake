package drake.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;


public class BlockingMessageMonitor implements LCMSubscriber {
  private final Map<String, byte[]> byte_arrays;
  private final Lock lock = new ReentrantLock();
  private final List<String> channels_with_new_data_avalailable = new ArrayList<String>();
  private final Condition new_data = lock.newCondition();

  public BlockingMessageMonitor() {
    byte_arrays = new HashMap<String, byte[]>();
  }

  @Override
  public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
    lock.lock();
    try {
      int available = ins.available();
      // TODO: is it OK to have a byte array that is too large?
      byte[] byte_array = byte_arrays.get(channel);
      if (byte_array == null || byte_array.length != available) {
        byte_array = new byte[available];
        byte_arrays.put(channel, byte_array);
      }
      try {
        ins.readFully(byte_array);
      } catch (IOException e) {
        System.err.println("MultipleMessageMonitor exception on channel " + channel);
        e.printStackTrace();
      }

      channels_with_new_data_avalailable.add(channel);
      new_data.signal();
    } finally {
      lock.unlock();
    }
  }
  
  public Map<String, byte[]> getMessages(long timeout_millis) {
    Map<String, byte[]> ret = new HashMap<String, byte[]>();
    try {
      lock.lock();
      TimeUnit unit = TimeUnit.MILLISECONDS;
      long timeout_nanos_remaining = unit.toNanos(timeout_millis);
      while (channels_with_new_data_avalailable.size() == 0 && timeout_nanos_remaining > 0l) {
        timeout_nanos_remaining = new_data.awaitNanos(timeout_nanos_remaining);
      }
      
      // make a deep copy of the byte arrays for the channels that have new data available
      for (int i = 0; i < channels_with_new_data_avalailable.size(); i++) {
        String channel = channels_with_new_data_avalailable.get(i);
        byte[] byte_array = byte_arrays.get(channel);
        ret.put(channel, Arrays.copyOf(byte_array, byte_array.length));
      }
      channels_with_new_data_avalailable.clear();

    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return ret;
  }
}
