
import numpy as np

class Logger(object):

    def __init__(self, console_print: bool = False) -> None:
        self.data_dict = {}
        self.data_shapes = {}
        self.console_print = console_print

    def __getitem__(self,key):
        return self.data_dict[key]

    def log(self, key, data):
        
        if (key in self.data_dict.keys()) is False:
            if self.console_print: 
                print(f'Logger added: {key}')
            self.data_dict[key] = data
            self.data_shapes[key] = np.shape(data)
        else:
            if self.data_shapes[key] != np.shape(data):
                raise Exception(
                    f'Logger exepted wrong data shape for key {key}:\n'
                    f'Need: {self.data_dict[key][0].shape}, exepted: {np.shape(data)}'    
                )
            self.data_dict[key] = np.vstack((self.data_dict[key], data))
