# Abstract base class for all single-pin IO objects


class io_base:
    def __init__(self, pin:int) -> None:
        self._pin = pin