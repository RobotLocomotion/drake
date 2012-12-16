package drake.util;

import java.util.*;

import javax.swing.*;
import javax.swing.event.*;
import javax.swing.table.*;
import javax.swing.tree.*;
import java.awt.*;
import java.awt.event.*;

import java.io.*;
import java.util.*;
import java.util.jar.*;
import java.util.zip.*;

import lcm.util.*;

import java.lang.reflect.*;

import lcm.lcm.*;

/** Searches classpath for objects that implement LCSpyPlugin using reflection. 
 * This is a direct copy of lcm.spy.LCMTypeDatabase, with an additional 
 * constructor added to handle a string classpath.  This was necessary 
 * because MATLAB's classpath is filled with a huge a huge number of classes,
 * which make the ClassDiscoverer inefficient and even break it.  
 **/
public class MyLCMTypeDatabase
{
    HashMap<Long, Class> classes = new HashMap<Long, Class>();

    public MyLCMTypeDatabase()
    {
        ClassDiscoverer.findClasses(new MyClassVisitor());
        System.out.println("Found "+classes.size()+" LCM types");
    }

    /** Given a colon-delimited list of jar files, iterate over the
     * classes in them.
     * @param cp The colon-deliimited classpath to search
     **/
    public MyLCMTypeDatabase(String cp)
    {
        ClassDiscoverer.findClasses(cp,new MyClassVisitor());
        System.out.println("Found "+classes.size()+" LCM types");
    }

    class MyClassVisitor implements ClassDiscoverer.ClassVisitor
    {
        public void classFound(String jar, Class cls)
        {
            try {
                Field[] fields = cls.getFields();

                for (Field f : fields) {
                    if (f.getName().equals("LCM_FINGERPRINT")) {
                        // it's a static member, we don't need an instance
                        long fingerprint = f.getLong(null);
                        classes.put(fingerprint, cls);
//                        System.out.printf("%016x : %s\n", fingerprint, cls);

                        break;
                    }
                }
            } catch (IllegalAccessException ex) {
                System.out.println("Bad LCM Type? "+ex);
            }
        }
    }

    public Class getClassByFingerprint(long fingerprint)
    {
//      System.out.printf("trying %016x\n", fingerprint);
      return classes.get(fingerprint);
    }
}
