from machine import ADC

# Light Sensor (LDR) Class
class LightSensor():
    '''
        Constants
    '''
    LIGHT_READING_UPPER_LIMIT = 20000
    LIGHT_READING_LOWER_LIMIT = 300

    '''
        Constructor
    '''
    def __init__(self, pin):
        self.adc_pin = ADC(pin)

    '''
        get_lux:
            This method returns the light level.
            The value is higher when there is less light,
            and lower when there is more light
    '''
    def get_lux(self):
        return self.adc_pin.read_u16()

    '''
        light_is_on:
            This method returns true or false, depending on the light level in the room.
            True should mean that the light switch has been flipped, and false means that it hasn't
            The value could change depending on the environment (open windows, time of day, etc)
    '''
    # We will have to fine tune this function
    def light_is_on(self):
        # get light level reading
        light_level = self.adc_pin.read_u16()
        # compare reading to lower and upper limit
        if (light_level > LIGHT_READING_LOWER_LIMIT && light_level < LIGHT_READING_UPPER_LIMIT):
            return false
        else:
            return true

# END_CLASS
