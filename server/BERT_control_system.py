""" BERT's Control system
"""

# Libraries
from BERT_client import BertClient
from read_control_data import ArduinoPy

# --------------------------------------------------------------------------------------

# initialize objects
bert = BertClient()
ardipy = ArduinoPy()


def activate_bert():
    """Activate video and data transmission between BERT and server"""

    # Connect to server
    bert.connect()

    # Read control data, if verified - send to server
    while True:
        try:
            control_data = ardipy.read_control_data_from_bert()
            bert.send_data_to_server(control_data)
        except Exception as error:
            print(error)


if __name__ == "__main__":
    activate_bert()
