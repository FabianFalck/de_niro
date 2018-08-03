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
            {
              "type": "service_account",
              "project_id": "fezzik-1516182462090",
              "private_key_id": "bfa6c2e6a6f7a789bf194c3a07b7b94f0d647652",
              "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQCpg8Fyv1n+BK0u\n7HnG7BKBlYY9xDoicaLyMWEWvN8i6KXLIIJNyvgDvuFdu4EEC0Onc63eBZeigvNF\nsDWF8kbcdATjwgphL5KP9EJKxjyZfKnIe8wACPgCnkUidzUoxQS4h1kfXPASXqnH\nNXocKC7Aof+LuiMugWiBUb+U6F7Tp6SGcSwDqO4Y+kUFgKzqTp1eoBAZD0He4lWq\n+U05oGiZXQ6otgcXvNJtEWNw/xwjOjPl0QdXinPUThu+L0pe9swas5Aa5JR3w5zg\nXXC8ou3P88Xtu36axHZtje7Y01gePvR+mwHd706PRoa6djHVbOKQIJJEzLR8jNkt\nkHgW05pRAgMBAAECggEALDTrBwS18FPB4kHTRDgeX2LiHkuFKaRe7TBJdIeO3Wi0\ni/OPZ8Aty7REy4/xhl91pj65sfPdZokl7h+U5biYGDe9arQYxt2EKDx36U4nUZO8\nJ0d5nOy52NzzhJh36YjvwTuy10YCaZf8SLI+hQdJofzdStAhCqi+2zK5dpje2e0q\nYzZAdRSS4AZ9Z6JxSoesCbI4OyWq4iUYBSGsbKkp0khjnmB3yt82tt5frB6H2wf8\nyt7VVeBNN7neCg84dDh6pfXGdk1+8KD3hRM+rUyiK7hvSWHFGNUFWuQaHj5OLFhM\n7usDh4jOvSg6LoFw554xJDAR5pzGOZiHuBxTpahLLQKBgQDbafz166i3i3xblGeT\nlXNo9t/tnymD8dppO0+AozMvhdA9UumRKimoR1Z0cXduTRIDg+6Np3MdCf0xcuOo\nHxTP3Xm9/iRhS7DFPBDpmaJpr5Gmh9MY4vyxOSsr85u/Tqm5ff+7aK2Geui7wqVQ\ndZQWcMDFJw2XngLNeJm2U9YLhwKBgQDFx716D9ZGCBn/dWJcUP4JWeQ0T2tlWFLE\n9s4hCD2Vabvpe/aEpZ4Lms1N5T1LpKPTSs7lKfpS8LwqcuFOxFcD8lOIYuGkwNud\nJkmBgvctaOpxdC59frW/i0UiJ4VzHWelc8sJET1QVz8ci7SrYCFIWYh7XhWtQgQd\n8Pe4GccRZwKBgFzOD7jU3KAF16vEsaqy5AXZpg2T7LNqcL9h3tbCMLxBFTta/n/w\nHX6X7fGCrMlyv9PM0fIIiaAWwYlQ1wodW2gwfXXnMOwWX8aNCXpFCXU6VZjiil9U\nYx1y2NfWSmI4m4Fh0fEq7XqcMiR4yAOoN+Ll/iNlyVH/+599C0cbXsWxAoGAenR2\nI1ok4A1qE1oZgXEAKMgIXD8EDqaF8L+i1p7kilsB5slC09Q7K5I4JU+XCPuyF0ON\nICRCghHdXybryzoTajUidSJL9RcXopAvGMn+wpXf+kjl8/t9ClBviHsRzDlbj/xG\nrKGI5fJu9/yj6yvEz6fnsgKJx1FFfUhSCCGluGcCgYBMjP1CVEiZiBvzpRmy4APK\nofuyFYJHTG9LhMApLvZrl9gTLviJsUkVEGT9hIljTAg8az/zUN5Zhc3n9hvamZi0\njnS5OGhrVJI61GLFLxnNFUfMSw9gsb5DB9vlgnlxfKssnbOsk3xJoVaEQLd3nuAY\n+KVWYvUSJpaQll6n0pyN5g==\n-----END PRIVATE KEY-----\n",
              "client_email": "starting-account-39gdz4an3nh5@fezzik-1516182462090.iam.gserviceaccount.com",
              "client_id": "103979675553226487787",
              "auth_uri": "https://accounts.google.com/o/oauth2/auth",
              "token_uri": "https://accounts.google.com/o/oauth2/token",
              "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
              "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/starting-account-39gdz4an3nh5%40fezzik-1516182462090.iam.gserviceaccount.com"
            }
            """,

            'list_of_warehouse_objects': ['coffee', 'water', 'apple juice'],  # When changed, also change fetch.gram definition

            'max_misattempts': 2,

            'sphinx_keyword_sensitivity': 0.001,
        },

    }
    return configs
