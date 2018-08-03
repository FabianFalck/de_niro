"""
Contains configurations for the speech recognition APIs

Author: Fabian Falck
Date: 05/18
"""


def create_configs():
    """
    Creating configs dictionary for the speech APIs
    :return: Dictionary
    """

    configs = {
        'speech_recognition_module': 'python_lib',  # Possible values: 'python_lib', 'amazon_echo'

        'configs_python_lib': {
            # 'sphinx' (for offline usage), 'google' (for online usage)
            'speech_recognition_API': 'sphinx',

            # r"""INSERT THE CONTENTS OF THE GOOGLE CLOUD SPEECH JSON CREDENTIALS FILE HERE"""
            # assign this content to the variable GOOGLE_CLOUD_SPEECH_CREDENTIALS
            # THE FOLLOWING CONTENT IS CONFIDENTIAL AND MAY NOT BE SHARED
            'GOOGLE_CLOUD_SPEECH_CREDENTIALS': r"""
            <insert your API key information as given by Google Cloud here>
            """,

            'list_of_warehouse_objects': ['coffee', 'water', 'apple juice'],  # When changed, also change fetch.gram definition

            'max_misattempts': 2,

            'sphinx_keyword_sensitivity': 0.001,
        },

    }
    return configs
