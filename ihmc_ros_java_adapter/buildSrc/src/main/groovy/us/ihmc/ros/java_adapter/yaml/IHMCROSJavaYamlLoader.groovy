package us.ihmc.ros.java_adapter.yaml

import org.yaml.snakeyaml.Dumper
import org.yaml.snakeyaml.DumperOptions
import org.yaml.snakeyaml.Loader
import org.yaml.snakeyaml.Yaml


class IHMCROSJavaYamlLoader {
    Yaml yaml

    IHMCROSJavaYamlLoader() {
        def dumperOptions = new DumperOptions()
        dumperOptions.setDefaultFlowStyle(DumperOptions.FlowStyle.BLOCK)
        yaml = new Yaml(new Loader(new IHMCROSJavaConfigurationConstructor()), new Dumper(new IHMCROSJavaConfigurationRepresenter(), dumperOptions));
    }

    def loadYaml(String pathToYaml) {
        return yaml.load(new FileReader(pathToYaml))
    }

    def dumpYaml(Object data, String pathToYaml) {
        yaml.dump(data, new FileWriter(pathToYaml))
    }
}
