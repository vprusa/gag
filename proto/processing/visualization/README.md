# GAG Processing visualization 

To run reference file exec:

```
mvn clean install exec:java -Dexec.mainClass="cz.gag.common.ProcessingApplet"  -DreplayPath=./output-test3.log
```

```
mvn clean install exec:java -Dexec.mainClass="cz.gag.common.ProcessingApplet" -DportNameRight="/dev/ttyUSB0" -DskipTests=true -DforceDataRequest=null | tee app.log
```
