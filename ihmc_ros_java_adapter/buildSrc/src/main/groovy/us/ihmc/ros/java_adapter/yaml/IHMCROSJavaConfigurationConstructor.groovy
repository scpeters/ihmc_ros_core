package us.ihmc.ros.java_adapter.yaml

import org.yaml.snakeyaml.constructor.AbstractConstruct
import org.yaml.snakeyaml.constructor.Constructor
import org.yaml.snakeyaml.nodes.MappingNode
import org.yaml.snakeyaml.nodes.Node
import org.yaml.snakeyaml.nodes.Tag
import us.ihmc.ros.java_adapter.IHMCROSJavaConfiguration
import us.ihmc.ros.java_adapter.IHMCROSJavaDependencyHolder
import us.ihmc.ros.java_adapter.IHMCROSJavaVMConfiguration

class IHMCROSJavaConfigurationConstructor extends Constructor {
    public static final String CORE_TAG = "!ihmc_ros_java_adapter"
    public static final String DEPENDENCIES_TAG = "dependencies"
    public static final String VM_TAG = "vmConfig"
    public static final String ARGS_TAG = "programArgs"

    public IHMCROSJavaConfigurationConstructor() {
        this.yamlConstructors.put(new Tag(CORE_TAG), new ConstructConfiguration())
    }

    private class ConstructConfiguration extends AbstractConstruct {

        @Override
        Object construct(Node node) {
            def configMap = constructMapping(node as MappingNode)
            IHMCROSJavaConfiguration yamlConfiguration = new IHMCROSJavaConfiguration(new IHMCROSJavaDependencyHolder(new ArrayList<String>()), new IHMCROSJavaVMConfiguration());

            yamlConfiguration.vmConfiguration.mainMethod = (configMap.get(VM_TAG) as Map<String, String>).get("mainMethod")
            yamlConfiguration.vmConfiguration.heapSize = (configMap.get(VM_TAG) as Map<String, String>).get("heapSize")
            yamlConfiguration.dependencyHolder.dependencies.addAll(configMap.get(DEPENDENCIES_TAG) as List<String>)
            yamlConfiguration.programArguments = configMap.get(ARGS_TAG) as ArrayList<String>

            return yamlConfiguration
        }
    }
}
