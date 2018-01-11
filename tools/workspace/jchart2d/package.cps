{
  "Cps-Version": "0.8.0",
  "Name": "jchart2d",
  "Description": "Minimalistic real-time charting library",
  "License": [
    "Apache-2.0",
    "GPL-2.0 WITH Classpath-exception-2.0",
    "LGPL-3.0+"
  ],
  "Version": "3.3.2",
  "Default-Components": [":jchart2d"],
  "Components": {
    "jide-oss": {
      "Type": "jar",
      "Location": "@prefix@/share/java/jide-oss-2.9.7.jar"
    },
    "commons-io": {
      "Type": "jar",
      "Location": "@prefix@/share/java/commons-io-1.3.1.jar"
    },
    "xmlgraphics-commons": {
      "Type": "jar",
      "Location": "@prefix@/share/java/xmlgraphics-commons-1.3.1.jar",
      "Requires": [":commons-io"]
    },
    "jchart2d": {
      "Type": "jar",
      "Location": "@prefix@/share/java/jchart2d-3.3.2.jar",
      "Requires": [
        ":jide-oss",
        ":xmlgraphics-commons"
      ]
    }
  }
}
