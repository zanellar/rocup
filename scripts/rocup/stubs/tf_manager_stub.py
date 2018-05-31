from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy


class TfManagerStub(object):
    DEFAULT_RECEIVER_NAME = "tf_manager"

    def __init__(self, namespace='common', sender_name=""):
        self.message_proxy = SimpleMessageProxy(name=namespace)
        self.sender_name = sender_name

    def save(self, tf_name, tf_parent, saving_name):
        message = SimpleMessage.messageFromDict({
            'receiver': TfManagerStub.DEFAULT_RECEIVER_NAME,
            'command': 'save',
            'tf_name': tf_name,
            'tf_parent': tf_parent,
            'saving_name': saving_name,
            'sender': self.sender_name
        })
        self.message_proxy.send(message)

    def delete(self, saving_name):
        message = SimpleMessage.messageFromDict({
            'receiver': TfManagerStub.DEFAULT_RECEIVER_NAME,
            'command': 'delete',
            'saving_name': saving_name,
            'sender': self.sender_name
        })
        self.message_proxy.send(message)

    def deleteAll(self):
        message = SimpleMessage.messageFromDict({
            'receiver': TfManagerStub.DEFAULT_RECEIVER_NAME,
            'command': 'delete_all',
            'sender': self.sender_name
        })
        self.message_proxy.send(message)

    def startPublish(self):
        message = SimpleMessage.messageFromDict({
            'receiver': TfManagerStub.DEFAULT_RECEIVER_NAME,
            'command': 'start_publish',
            'sender': self.sender_name
        })
        self.message_proxy.send(message)

    def stopPublish(self):
        message = SimpleMessage.messageFromDict({
            'receiver': TfManagerStub.DEFAULT_RECEIVER_NAME,
            'command': 'stop_publish',
            'sender': self.sender_name
        })
        self.message_proxy.send(message)

    def update(self):
        message = SimpleMessage.messageFromDict({
            'receiver': TfManagerStub.DEFAULT_RECEIVER_NAME,
            'command': 'update',
            'sender': self.sender_name
        })
        self.message_proxy.send(message)
