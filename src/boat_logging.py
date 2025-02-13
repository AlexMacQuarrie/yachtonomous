# Internal
from tools import arr


class boat_logger:
    def __init__(self, log_en:bool, log_file_name:str) -> None:
        self.__log_en    = log_en
        self.__file_name = log_file_name
        print('Starting test')
        self.__write_header('x_x,x_y,x_t,x_g,x_p,x_e,'
                            'x_d_x,x_d_y,x_d_t,x_d_g,x_d_p,x_d_e,'
                            'u_e,u_p,u_a_e,u_a_p,'
                            't')

    def end(self) -> None:
        ''' End of logging '''
        print('Test complete')

    def log_results(self, x_hat:arr, x_d:arr, u:arr, u_act:arr, t:float) -> None:
        ''' Log states/inputs '''
        self.__write_data(
            self.__to_csv(x_hat) +
            self.__to_csv(x_d)   +
            self.__to_csv(u)     +
            self.__to_csv(u_act) +
            str(t)
        )

    def __to_csv(self, a:arr) -> str:
        ''' Convert states/inputs to csv string '''
        csv = ''
        for elem in a:
            csv += str(elem) + ','
        return csv

    def __write_header(self, msg:str) -> None:
        ''' Write header, override contents '''
        self.__write(msg, 'w')

    def __write_data(self, msg:str) -> None:
        ''' Write data, append contents '''
        self.__write(msg, 'a')

    def __write(self, msg:str, mode:str) -> None:
        ''' Write to csv file '''
        if self.__log_en:
            with open(self.__file_name, mode) as log_file:
                log_file.write(msg + '\n')