# Entrypoint for Yachtonomous
from algo.control import sailboat
from tools.pin_consts import pin_consts


def main() -> None:
    ''' Entrypoint '''
    print('Starting Yachtononmous')

    # Create a boat with some pins
    boat = sailboat(
        pin_consts.BOAT_PIN,
        pin_consts.DEST_PIN,
        pin_consts.WIND_PIN,
        pin_consts.SAIL_PIN,
        pin_consts.RUDDER_PIN,
        pin_consts.SWITCH_PIN,
        *pin_consts.LED_PINS
    )

    # Do stuff with boat once
    boat.nav.plot_course()

    # Do stuff with boat repeatedly
    while True:
        boat.nav.plot_course()

        if True:   # TODO: break when at dest OR timeout OR error state (catch w/ try/except)
            break

    print('Completed Yachtonomous')


if __name__ == '__main__':
    main()