class ConfigException(Exception):

    def __init__(self, error_message=None):
        if not error_message:
            error_message = 'The config file is missing some necessary fields'

    def __repr__(self):
        return error_message
