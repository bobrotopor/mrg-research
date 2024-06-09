
import numpy as np

class Logger(object):

    def __init__(self) -> None:
        self.data_dict = {}
        self.data_shapes = {}

    def __getitem__(self,key):
        return self.data_dict[key]

    def log(self, key, data):
        
        if (key in self.data_dict.keys()) is False:
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
            

    