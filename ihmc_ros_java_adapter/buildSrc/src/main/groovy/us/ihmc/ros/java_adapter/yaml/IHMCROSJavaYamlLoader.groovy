package us.ihmc.ros.java_adapter.yaml

import org.yaml.snakeyaml.Yaml


class IHMCROSJavaYamlLoader {
    Yaml yaml

    IHMCROSJavaYamlLoader() {
        yaml = new Yaml(new IHMCROSJavaConfigurationConstructor(), new IHMCROSJavaConfigurationRepresenter());
    }

    def loadYaml(String pathToYaml) {
        return yaml.load(new FileReader(pathToYaml))
    }
}
