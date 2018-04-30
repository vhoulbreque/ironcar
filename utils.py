class ConfigException(Exception):
    """Handles the Exceptions concerning the Configuration file such as :
        - absence of the file
        - absence of one of the necessary fields
    """

    def __init__(self, error_message=None):
        if not error_message:
            error_message = 'The config file is missing some necessary fields'

    def __repr__(self):
        return error_message


class CameraException(Exception):
    """Handles the Exceptions concerning the PiCamera"""

    def __init__(self, error_message=None):
        if not error_message:
            error_message = 'The camera cannot be loaded'

    def __repr__(self):
        return error_message
