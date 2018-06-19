

class Config():
    """ class for loading and holding the config parameters for both the heatmap and slam map
    """
    def __init__(self, config_filepath=""):
        # variables
        self.map_width = 0.0 # in meters
        self.map_height = 0.0 # in meters
        self.map_resolution = 0.0 # in meters
        self.slam_origin = (0, 0) # map indecies
        self.slam_rotation = 0 # in degrees
        self.slam_uncertainty_cutoff = 0 # in slam map values

        if config_filepath != "":
            self.load_from_file(config_filepath)

    def load_from_file(self, filepath):
        """ load the config parameters from a file

        input:
            filepath - filepath to the config file
        """
        config_file = open(filepath, "r")
        config_dict = {}

        # load everything from the file
        line = "none"
        while line:
            line = config_file.readline()

            # check for lines of only whitespace
            if len(line.strip()) == 0:
                continue

            # process this line
            line_list = line.split("=")
            key = line_list[0].strip()
            value = eval(line_list[1].strip())

            config_dict[key] = value

        # load from the dictionary into the variables
        self.map_width = config_dict["map_width"]
        self.map_height = config_dict["map_height"]
        self.map_resolution = config_dict["map_resolution"]
        self.slam_origin = config_dict["slam_origin"]
        self.slam_rotation = config_dict["slam_rotation"]
        self.slam_uncertainty_cutoff = config_dict["slam_uncertainty_cutoff"]
