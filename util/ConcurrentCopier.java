/*
 *   Copyright 2014 Florida Institute for Human and Machine Cognition (IHMC)
 *    
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    
 *    http://www.apache.org/licenses/LICENSE-2.0
 *    
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *    
 *    Written by Alex Lesman with assistance from IHMC team members
 *    
 *    Modified by Twan Koolen:
 *    * changed package
 *    
 */
package drake.util;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * 
 * Class to copy data from one producer thread to one consumer thread guaranteeing atomicity.
 * This class is lock-free, non-blocking and garbage-free
 *   
 *   Only one producer and one consumer are supported
 *   
 * 
 * @author Jesper Smith
 * 
 * @param <T> object
 *
 */
public class ConcurrentCopier<T>
{
   private static final int NEXT_OBJECT_TO_READ_MASK = 0xC;
   private static final int CURRENTLY_BEING_READ_MASK = 0x3;
   private static final int INITIAL_STATE = 0xC;
   
   
   public final T[] buffer;
   
   private int currentlyBeingWritten = -1;
   
   /*
    * State bitmask integer
    * 
    * NEXT_OBJECT_TO_READ_MASK : nextObjectToRead
    * CURRENTLY_BEING_READ_MASK : currentlyBeingRead
    */
   private final AtomicInteger state = new AtomicInteger(); 
   
   
   @SuppressWarnings("unchecked")
   public ConcurrentCopier(Builder<? extends T> classBuilder)
   {
      buffer = (T[]) new Object[3];

      for (int i = 0; i < 3; i++)
      {
         buffer[i] = classBuilder.newInstance();
      }
      
      state.set(INITIAL_STATE);
   }
   
   public T getCopyForReading()
   {      
      while(true)
      {
         int currentState = state.get();
         if (currentState == INITIAL_STATE) 
         {
            return null;
         }
         
         int nextObjectToRead = (currentState & NEXT_OBJECT_TO_READ_MASK) >> 2;
         int newState = (currentState & NEXT_OBJECT_TO_READ_MASK) | (nextObjectToRead);
         if(state.compareAndSet(currentState, newState))
         {
            return buffer[nextObjectToRead];
         }
      }
   }
   
   // returns an index that is not beingRead or nextToRead
   private int getNextWriteIndex(int currentState)
   {
      switch(currentState)
      {
      case 0x0:
         return 0x1;
      case 0x1:
         return 0x2;
      case 0x2:
         return 0x1;
      case 0x4:
         return 0x2;
      case 0x5:
         return 0x0;
      case 0x6:
         return 0x0;
      case 0x8:
         return 0x1;
      case 0x9:
         return 0x0;
      case 0xA:
         return 0x0;
      case INITIAL_STATE:
         return 0x1;
      default:
         throw new RuntimeException("Invalid Copier State: " + currentState);
      }
      
   }
   
   public T getCopyForWriting()
   {
      currentlyBeingWritten = getNextWriteIndex(state.get());
      return buffer[currentlyBeingWritten];
   }
   
   
   /**
    * Commit write such that getCopyForWriting returns the newest copy
    */
   public void commit()
   {
      // Updating nextObjectToRead(state & NEXT_OBJECT_TO_READ_MASK) to currentlyBeingWritten 
      while(true)
      {
         int currentState = state.get();
         int newState = (currentState & CURRENTLY_BEING_READ_MASK) | (currentlyBeingWritten << 2);
         if(state.compareAndSet(currentState, newState))
         {
            break;
         }
      }
   }
}
