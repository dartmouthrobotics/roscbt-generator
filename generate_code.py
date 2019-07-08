import xml.etree.ElementTree
import subprocess
import os

'''This class parses a configuration file that contains all the settings that are required by the communication 
simulator. Some of the major settings include: 
- shared topics: topics that are being shared by the team of robots 
- topic remaps: In cases where robots publish messages to common topic, it may be necessary to remap the topic so that 
each robot publishes and/or listens to a unique topic 
- topology map: Communication topology map defines the communication topologies that need to be simulated. 
For instance, communication among a team of 10 robots can be designed such that robot 1 communicates with 2,3,4
whereas robot 5 communicates with only 6,7,8,9 and 10. This mapping can be specified here 

- robot_ids: A list of ids for all the robots in the multirobot system 

@Input: configuration file (e.g. configuration.xml): This has to be in xml format. see sample for details
@Output:
    - roscbt.launch: This is a launch file that implements the topic mappings, which allow the simulator to control message exchange among robots according to the defined topologies
    - roscbt_config.yaml: This is a param file from which the communication simulator reads the configurations

@Author: Kizito Masaba, Alberto Quattrini Li
         Rlab@Dartmouth
'''

class RoscbtGenerator:
    def __init__(self, config_filepath, destination_folder):
        """

        :param config_filepath: configuration filename should be in xml format
        :param destination_folder:  destination folder to which the generated files are
        saved
       """
        self.config_filename=config_filepath
        self.destination_folder = None
        if destination_folder and destination_folder[-1]=='/':
            self.destination_folder=destination_folder[:-1]
        else:
            self.destination_folder=destination_folder

        # variables that hold the configuration parameters while parsing the configuration file
        self.shared_topics=[]
        self.topic_remaps=[]
        self.unique_robot_ids=[]
        self.topology_map={}
        self.package_name=None


    """
    This parses the cofinguration file designed by the user
    """
    def parse_configuration_file(self):
        package_element = xml.etree.ElementTree.parse(self.config_filename).getroot()
        package_children = list(package_element)
        self.package_name = package_element.get("to-pkg")
        self.unique_robot_ids = []
        for elem in package_children:
            if elem.tag == "topics":
                topics = list(elem)
                topic_ns = elem.get("ns")
                for topic in topics:
                    topic_name = topic.get("name")
                    topic_pkg = topic.get("pkg")
                    to_remap = int(topic.get("remap"))
                    message_pkg=topic.get("message_pkg")
                    topic_message_type = topic.get("message_type")
                    self.shared_topics.append({"name": topic_name, "pkg": topic_pkg, "message_type": topic_message_type,"message_pkg": message_pkg,'remap':to_remap, 'ns': topic_ns})
    
            elif elem.tag == "remaps":
                remaps = list(elem)
                if len(remaps) > 0:
                    for remap in remaps:
                        if remap.tag == 'topic':
                            topic_name = remap.get('name')
                            to_remap = int(remap.get("remap"))
                            message_pkg = remap.get("message_pkg")
                            message_type = remap.get("message_type")
                            topic_pkg = remap.get('pkg')
                            topic_from = remap.get('from')
                            topic_to = remap.get('to')
                            self.topic_remaps.append({"name":topic_name,"pkg": topic_pkg, "from": topic_from, "to": topic_to,'message_type':message_type,'message_pkg':message_pkg,'remap':to_remap})
            elif elem.tag == "topology":
                all_robots = []
                topologies = list(elem)
                for topology in topologies:
                    robot_map = list(topology)
                    map_id = topology.get("id")
                    map_comm_model = topology.get("default_comm_model")
                    robot_ids = []
                    for robot in robot_map:
                        robot_id = int(robot.text)
                        robot_ids.append(robot_id)
                    self.topology_map[map_id] = {'comm_model': map_comm_model, 'robots': robot_ids}
                    all_robots += robot_ids
                self.unique_robot_ids = set(all_robots)

    '''
       This generates the launch file using the configuration parameters
       @Input  destination folder full path: This is where you wha
       @Output roscbt.launch: this is the roscbt launch file
               roscbt_config.yaml: this saves the configuration parameters, which are read by the communication simulator
    '''
    def generate_launch_file(self):
        sorted_topics={}
        remapped_topics=[]
        for shared_topic in self.shared_topics:
            if shared_topic['remap']==1:
                if shared_topic['pkg'] in sorted_topics:
                    sorted_topics[shared_topic['pkg']].append(shared_topic)
                else:
                    sorted_topics[shared_topic['pkg']]=[shared_topic]

        for shared_topic in self.topic_remaps:
            if shared_topic['remap'] == 1:
                if shared_topic['pkg'] in sorted_topics:
                    sorted_topics[shared_topic['pkg']].append(shared_topic)
                else:
                    sorted_topics[shared_topic['pkg']]=[shared_topic]
                remapped_topics.append(shared_topic['from'])

        with open("{}/launch/roscbt.launch".format(self.destination_folder), "w") as fd:
            fd.write('<?xml version="1.0"?>\n')
            fd.write('<launch>\n')
            ns_wrappers={}

            for pkg,topics in sorted_topics.items():
                for ns in self.unique_robot_ids:
                    full_ns = "robot_{}".format(ns)
                    group = '<group ns="{}">\n'.format(full_ns)
                    ns_wrappers[ns] = group

                    robot_str = '<node pkg="{}" name="Mapper" type="mapper">\n'.format(pkg)
                    for topic in topics:
                        if topic['name'] not in remapped_topics:
                            if topic['remap']==1:
                                robot_str += '<remap from="{}" to="/roscbt/{}/{}" />\n'.format(topic['name'], full_ns, topic['name'])
                        else:
                            robot_str += '<remap from="{}" to="{}" />\n'.format(topic['from'], topic['to'])
                    robot_str =ns_wrappers[ns]+robot_str+ '</node>\n</group>\n'
                    fd.write(robot_str)

            fd.write('<node name="roscbt" pkg="{}" type="roscbt.py" respawn="true">\n'.format(self.package_name))
            fd.write('<rosparam file="$(find {})/param/roscbt_config.yaml"/>\n'.format(self.package_name))
            fd.write('</node>\n')

            # close launch tag and close file
            fd.write('</launch>\n')
            fd.close()

    """
    This generates the yaml file that is read by the communication simulator
    """
    def generate_param_file(self):
        robot_maps=[]
        for k,v in self.topology_map.items():
            robots=v['robots']
            for r in robots:
                conns=[p for p in robots if r!=p]
                robot_maps.append({'id':r,'comm_model': v['comm_model'], 'connections':conns})
        with open("{}/param/roscbt_config.yaml".format(self.destination_folder), "w") as fd:
            fd.write('robot_ids: {}\n'.format(list(self.unique_robot_ids)))
            fd.write('robot_map: {}\n'.format(robot_maps))
            fd.write('topics: {}\n'.format(self.shared_topics))
            fd.write('topic_remaps: {}\n'.format(self.topic_remaps))
            fd.close()

    def copy_roscbt(self):
        current_dir=os.getcwd()
        mv_command = 'cp {}/roscbt.py {}/scripts/roscbt.py'.format(current_dir, self.destination_folder)
        cmd_file='command.sh'
        with open(cmd_file, 'w') as fd:
            fd.write('#!/bin/bash\n')
            fd.write(mv_command)
            fd.close()

        subprocess.call(['chmod', '0777', cmd_file])
        subprocess.Popen(["bash", cmd_file])


if __name__ == '__main__':
    roscbt_gen=RoscbtGenerator('configuration.xml','/home/masaba/catkin_ws/src/finalproject')
    roscbt_gen.parse_configuration_file()
    roscbt_gen.generate_launch_file()
    roscbt_gen.generate_param_file()
    roscbt_gen.copy_roscbt()