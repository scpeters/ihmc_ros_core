package us.ihmc.ros.java_adapter

import groovy.transform.Canonical
import org.yaml.snakeyaml.Yaml
import org.yaml.snakeyaml.constructor.AbstractConstruct
import org.yaml.snakeyaml.constructor.Constructor
import org.yaml.snakeyaml.nodes.MappingNode
import org.yaml.snakeyaml.nodes.Node
import org.yaml.snakeyaml.nodes.SequenceNode
import org.yaml.snakeyaml.nodes.Tag
import org.yaml.snakeyaml.representer.Represent
import org.yaml.snakeyaml.representer.Representer

class IHMCROSJavaYamlLoader {
    Yaml yaml

    IHMCROSJavaYamlLoader() {
        yaml = new Yaml(new IHMCRosJavaConfigurationConstructor(), new IHMCRosJavaConfigurationRepresenter());
    }

    def loadYaml(String pathToYaml) {
        return yaml.load(new FileReader(pathToYaml))
    }

    def dumpYaml(String pathToYaml) {
        def deps = new ArrayList<String>()
        deps.add("us.ihmc:Atlas:0.7.4")


        def tmpDepHolder = new IHMCRosJavaDependencyHolder(deps)
        def tmpVmConfig = new IHMCRosJavaVMConfiguration("us.ihmc.atlas.AtlasFlatGroundWalkingTrack", "4g")

        def dumpObject = new IHMCRosJavaYamlConfiguration(tmpDepHolder, tmpVmConfig)
//        def dumpStuff = new ArrayList<Object>();
//        dumpStuff.add(tmpVmConfig)
//        dumpStuff.add(tmpDepHolder)
        yaml.dump(dumpObject, new FileWriter(pathToYaml))
    }
}

@Canonical
class IHMCRosJavaYamlConfiguration {
    IHMCRosJavaDependencyHolder dependencyHolder
    IHMCRosJavaVMConfiguration vmConfiguration
}

class IHMCRosJavaConfigurationConstructor extends Constructor {
    public static final String CORE_TAG = "!ihmc_ros_java_adapter"
    public static final String DEPENDENCIES_TAG = "!dependencies"
    public static final String VM_TAG = "!vmConfig"

    public IHMCRosJavaConfigurationConstructor(){
        this.yamlConstructors.put(new Tag(CORE_TAG), new ConstructConfiguration())
        this.yamlConstructors.put(new Tag(VM_TAG), new ConstructVMConfiguration())
        this.yamlConstructors.put(new Tag(DEPENDENCIES_TAG), new ConstructDependencies())
    }

    private class ConstructConfiguration extends AbstractConstruct {

        @Override
        Object construct(Node node) {
            def configMap = constructSequence(node as SequenceNode)
            IHMCRosJavaYamlConfiguration yamlConfiguration = new IHMCRosJavaYamlConfiguration();

            configMap.forEach({ yamlObject ->
                if(yamlObject instanceof IHMCRosJavaVMConfiguration)
                    yamlConfiguration.vmConfiguration = yamlObject as IHMCRosJavaVMConfiguration
                else if(yamlObject instanceof  IHMCRosJavaDependencyHolder)
                    yamlConfiguration.dependencyHolder = yamlObject as IHMCRosJavaDependencyHolder
            })

            return yamlConfiguration
        }
    }

    private class ConstructVMConfiguration extends AbstractConstruct {
        @Override
        Object construct(Node node) {
            Map<String, String> mapping = constructMapping(node as MappingNode) as Map<String, String>;
            def heapSize = mapping.get("heapSize")
            def mainMethod = mapping.get("mainMethod")

            return new IHMCRosJavaVMConfiguration(mainMethod, heapSize)
        }
    }

    private class ConstructDependencies extends AbstractConstruct {
        @Override
        Object construct(Node node) {
            def dependenciesList = constructSequence(node as SequenceNode) as List<String>
            return new IHMCRosJavaDependencyHolder(dependenciesList)
        }
    }
}

class IHMCRosJavaConfigurationRepresenter extends Representer {

    IHMCRosJavaConfigurationRepresenter() {
        this.representers.put(IHMCRosJavaYamlConfiguration.class, new RepresentConfiguration())
        this.representers.put(IHMCRosJavaDependencyHolder.class, new RepresentDependencies())
        this.representers.put(IHMCRosJavaVMConfiguration.class, new RepresentVMConfiguration())
    }

    private class RepresentConfiguration implements Represent {

        @Override
        Node representData(Object o) {
            def config = o as IHMCRosJavaYamlConfiguration
            def depHolder = config.dependencyHolder
            def vmConfig = config.vmConfiguration

            def mapping = new ArrayList<Object>()

            mapping.add(vmConfig)
            mapping.add(depHolder)

            return representSequence(new Tag(IHMCRosJavaConfigurationConstructor.CORE_TAG), mapping, false)
        }
    }

    private class RepresentDependencies implements Represent {
        @Override
        Node representData(Object o) {
            def dependencyHolder = o as IHMCRosJavaDependencyHolder
            return representSequence(new Tag(IHMCRosJavaConfigurationConstructor.DEPENDENCIES_TAG), dependencyHolder.dependencies, false)
        }
    }

    private class RepresentVMConfiguration implements Represent {
        @Override
        Node representData(Object o) {
            def vmConfig = o as IHMCRosJavaVMConfiguration
            def stringMapping = new HashMap<String, String>()

            stringMapping.put("mainMethod", vmConfig.mainMethod)
            stringMapping.put("heapSize", vmConfig.heapSize);

            return representMapping(new Tag(IHMCRosJavaConfigurationConstructor.VM_TAG), stringMapping, false)
        }
    }
}

@Canonical
class IHMCRosJavaDependencyHolder {
    List<String> dependencies
}

@Canonical
class IHMCRosJavaVMConfiguration {
    String mainMethod
    String heapSize
}