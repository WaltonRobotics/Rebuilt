https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/profiling-with-visualvm.html

in `build.gradle`, replace 

```
frcJava(getArtifactTypeClass('FRCJavaArtifact')) {
}
```

with

```
frcJava(getArtifactTypeClass('FRCJavaArtifact')) {
    // Enable VisualVM connection - TURN THIS OFF WHEN NOT USING VISUALVM PLEASSSSEEEEEEEE
    jvmArgs.add("-Dcom.sun.management.jmxremote=true")
    jvmArgs.add("-Dcom.sun.management.jmxremote.port=1198")
    jvmArgs.add("-Dcom.sun.management.jmxremote.local.only=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.ssl=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.authenticate=false")
    jvmArgs.add("-Djava.rmi.server.hostname=10.29.74.2")
}
```

in the terminal

```
cd "C:\Users\Public\wpilib\visualvm_221\bin"
./visualvm --jdkhome "C:\Users\Public\wpilib\2026\jdk"
```