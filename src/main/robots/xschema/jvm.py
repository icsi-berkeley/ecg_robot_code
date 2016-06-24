"""Created on 2/19/2015 by Steve Doubleday
"""

from jpype import *
import os.path


class PetriNetJVM:
    """ interface to a Petri net
    """
    alreadyRunning = False
    def __init__(self):
        if not PetriNetJVM.alreadyRunning:
            cp = '-Djava.class.path=$M2REPO/uk/ac/imperial/pipe-core/1.0.3-SNAPSHOT/pipe-core-1.0.3-SNAPSHOT.jar:$M2REPO/uk/ac/imperial/pipe-markov-chain/1.0.2-SNAPSHOT/pipe-markov-chain-1.0.2-SNAPSHOT.jar:$M2REPO/org/apache/logging/log4j/log4j-api/2.3/log4j-api-2.3.jar:$M2REPO/org/apache/logging/log4j/log4j-core/2.3/log4j-core-2.3.jar:$M2REPO/com/esotericsoftware/kryo/kryo/2.24.0/kryo-2.24.0.jar:$M2REPO/com/esotericsoftware/minlog/minlog/1.2/minlog-1.2.jar:$M2REPO/org/objenesis/objenesis/2.1/objenesis-2.1.jar:$M2REPO/org/codehaus/jackson/jackson-mapper-asl/1.9.13/jackson-mapper-asl-1.9.13.jar:$M2REPO/org/codehaus/jackson/jackson-core-asl/1.9.13/jackson-core-asl-1.9.13.jar:$M2REPO/commons-logging/commons-logging/1.1.3/commons-logging-1.1.3.jar:$M2REPO/commons-beanutils/commons-beanutils/1.8.3/commons-beanutils-1.8.3.jar:$M2REPO/commons-collections/commons-collections/3.0/commons-collections-3.0.jar:$M2REPO/commons-io/commons-io/1.3.2/commons-io-1.3.2.jar:$M2REPO/com/google/guava/guava/17.0/guava-17.0.jar:$M2REPO/de/twentyeleven/skysail/jgraphx-osgi/1.10.3.1/jgraphx-osgi-1.10.3.1.jar:$M2REPO/org/antlr/antlr4/4.2/antlr4-4.2.jar:$M2REPO/org/antlr/antlr4-runtime/4.2/antlr4-runtime-4.2.jar:$M2REPO/org/abego/treelayout/org.abego.treelayout.core/1.0.1/org.abego.treelayout.core-1.0.1.jar:$M2REPO/org/antlr/antlr4-annotations/4.2/antlr4-annotations-4.2.jar:$M2REPO/org/antlr/antlr-runtime/3.5/antlr-runtime-3.5.jar:$M2REPO/org/antlr/stringtemplate/3.2.1/stringtemplate-3.2.1.jar:$M2REPO/antlr/antlr/2.7.7/antlr-2.7.7.jar:$M2REPO/org/antlr/ST4/4.0.7/ST4-4.0.7.jar:$M2REPO/javax/json/javax.json-api/1.0/javax.json-api-1.0.jar:$M2REPO/org/glassfish/javax.json/1.0.4/javax.json-1.0.4.jar:$M2REPO/edu/berkeley/icsi/robot-xnet/1.0.1-SNAPSHOT/robot-xnet-1.0.1-SNAPSHOT.jar'
            expcp = os.path.expandvars(cp)
            log4jdir = '-Dlog4j.configurationFile=${PWD}/src/main/log4j2.xml'
            explog4jdir = os.path.expandvars(log4jdir)
            #print(explog4jdir)
            startJVM(getDefaultJVMPath(), '-ea', '-Djava.awt.headless=true' ,explog4jdir, expcp)
            PetriNetJVM.alreadyRunning = True
            #print('JVM now Running')
        else:
            pass
            #print('JVM already running')     
    
