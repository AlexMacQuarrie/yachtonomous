# Internal
from tools import arr


class boat_logger:
    def __init__(self, log_en: bool, log_file_name: str) -> None:
        self.__log_en    = log_en
        self.__file_name = log_file_name

        if self.__log_en:
            print('Open logger')
            self.__log_file = open(self.__file_name, 'w')
            self.__write('x_x,x_y,x_t,x_g,x_p,x_e,'
                         'x_d_x,x_d_y,x_d_t,x_d_g,x_d_p,x_d_e,'
                         'u_e,u_p,u_a_e,u_a_p,'
                         't')

    def end(self) -> None:
        ''' Close the log file when logging is done '''
        if self.__log_en:
            print('Logger closed')
            self.__log_file.close()

    def log_results(self, x_hat: arr, x_d: arr, u: arr, u_act: arr, t: float) -> None:
        ''' Log states/inputs '''
        self.__write(
            self.__to_csv(x_hat) +
            self.__to_csv(x_d)   +
            self.__to_csv(u)     +
            self.__to_csv(u_act) +
            str(t)
        )

    def __to_csv(self, a: arr) -> str:
        ''' Convert states/inputs to CSV string '''
        return ','.join(map(str, a)) + ','

    def __write(self, msg: str) -> None:
        ''' Write to CSV file '''
        if self.__log_en:
            self.__log_file.write(msg + '\n')