import toml

# with open('../data/delivery.toml', 'r') as fp:
config_raw    = toml.load('../data/delivery.toml')
speech_params = toml.load('../data/speech.toml')
