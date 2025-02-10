# Internal
from tools import arr


class boat_logger:
    def __init__(self, log_en:bool) -> None:
        self.log_en = log_en
        self.__write_message('Starting test')
        if self.log_en:
            self.__write_message('--- PLACEHOLDER HEADER ---')

    @classmethod
    def __write_message(cls, msg:str) -> None:
        print(msg)

    def end(self) -> None:
        self.__write_message('Test complete')

    def log_results(self, x_hat:arr, x_d:arr, u:arr, u_act:arr) -> None:
        if self.log_en:
            self.__write_message(str(x_hat)+' '+str(x_d)+' '+str(u)+' '+str(u_act))