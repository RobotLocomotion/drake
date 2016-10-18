package drake.matlab.util;

import java.io.*;
import java.lang.*;
import java.lang.reflect.*;
import lcm.lcm.*;

/** 
 * Monitors a particular lcm message type (specified in the constructor)
 * and maintains the most up-to-date instance of that type.  
 *
 * The lcm message type must have a .timestamp field.  
 % If messages arrive out of order, it will keep the message with 
 * the largest timestamp.  If no message is received for timeout seconds
 * then the timestamp history is reset (e.g, it's assumed that the channel 
 * went dead and so accepts a time starting back at zero).  
 * The default value of this timeout is 1 second.
 *
 * Note that this is implemented with reflection instead of 
 * generics (e.g. MessageMonitor<lcmtype>) because the binding needs to 
 * happen at run-time (e.g. from MATLAB).
 *
 **/

public class MessageMonitor implements LCMSubscriber
{
  long m_last_timestamp;
  long m_time_of_last_message;
  long m_reset_time=1000;
  Field m_timestamp_field;
  byte[] m_data;
  Constructor<?> m_lcmtype_constructor=null;
  boolean m_has_new_message = false;
  
  public MessageMonitor(Class<?> lcmtype_class, String timestamp_field)
  {
    m_last_timestamp = -m_reset_time;
    m_time_of_last_message = System.currentTimeMillis();
    m_data = null;
    
    boolean hasTimestamp = true;
    try {
      m_timestamp_field = lcmtype_class.getField(timestamp_field);
    } catch (NoSuchFieldException ex) {
      hasTimestamp = false;
    }
    if (m_timestamp_field==null || hasTimestamp==false) {
      System.out.println("couldn't find the timestamp field '"+timestamp_field+"' in this LCM type");
      for (Field field : lcmtype_class.getFields()) {
        System.out.printf("Field name: %s%n", field.getName());
      }
    }
    
    if (m_timestamp_field.getType().equals(Long.TYPE)==false) {
      System.out.println("timestamp field is a "+m_timestamp_field.toString());
      System.out.println("timestamp field must be a long");
    }    

    try {
      Class[] params = new Class[1]; 
      params[0]=byte[].class; 
      m_lcmtype_constructor = lcmtype_class.getConstructor(params);
    } catch (NoSuchMethodException ex) {
      System.out.println("couldn't find constructor");
    }
  }
  
  public MessageMonitor(LCMEncodable lcmtype, String timestamp_field)
  {
    this(lcmtype.getClass(),timestamp_field);
  }

  
  public void setResetTime(long ms)
  {
    m_reset_time = ms;
  }

  public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
  {
    try {
      byte[] data = new byte[dins.available()];
      dins.readFully(data);
      Object msg = m_lcmtype_constructor.newInstance(data);
      long timestamp = m_timestamp_field.getLong(msg);
      long systime = System.currentTimeMillis();
      // include a 1 second timeout
      if (timestamp>=m_last_timestamp || systime-m_time_of_last_message>=m_reset_time) {
        m_data = data.clone();
        m_has_new_message = true;
        m_last_timestamp = timestamp;
//        System.out.println(timestamp);
      }
      m_time_of_last_message = systime;
      notifyAll();
    } catch (IOException ex) {
      System.out.println("MessageMonitor " + ex + " on channel " + channel);
    } catch (InstantiationException ex) {
      System.out.println("MessageMonitor " + ex + " on channel " + channel);
    } catch (IllegalAccessException ex) {
      System.out.println("MessageMonitor " + ex + " on channel " + channel);
    } catch (InvocationTargetException ex) {
      System.out.println("MessageMonitor " + ex + " on channel " + channel);
    }
  }
    
  public synchronized byte[] getNextMessage(long timeout_ms)
  {
    if (m_has_new_message) {
      m_has_new_message = false;
      return m_data;
    }
    
    if(timeout_ms == 0)
      return null;
    
    try {
      if(timeout_ms > 0)
        wait(timeout_ms);
      else
        wait();
      
      if (m_has_new_message) {
        m_has_new_message = false;
        return m_data;
      }
    } catch (InterruptedException ex) { }
    
    return null;
  }
  
  public synchronized byte[] getNextMessage()
  {
    return getNextMessage(-1);
  }
  
  public synchronized long getLastTimestamp()
  {
    return m_last_timestamp;
  }
  
  public synchronized void waitUntilTimestamp(long timestamp)
  {
    try { 
      while (m_last_timestamp<timestamp)
        wait();
    } catch (InterruptedException ex) {}
  }
  
  public synchronized boolean waitUntilTimestamp(long timestamp, long timeout_ms)
  {
    if (m_last_timestamp>=timestamp)
      return true;
    try {
      wait(timeout_ms);
    } catch (InterruptedException ex) {}
    //    System.out.println(m_last_timestamp-timestamp);
    return (m_last_timestamp>=timestamp);
  }
  
  public synchronized byte[] getMessage()
  {
  	m_has_new_message = false;
  	return m_data;
  }
  
  public synchronized void markAsRead()
  {
    m_has_new_message = false;
  }
  
  public synchronized void markAsReadBefore(long timestamp)
  {
    if (m_last_timestamp<timestamp)
      m_has_new_message = false;
  }
  
    /*
  public static void main(String[] args) throws InterruptedException
  {
    MessageMonitor m = new MessageMonitor(new drc.robot_state_t(),"utime");
    LCM lcm = LCM.getSingleton();
    lcm.subscribe("EST_ROBOT_STATE",m);
    
    while(true)
      Thread.sleep(1000);
  }
    */  
}
