#!/usr/bin/env py -3
from my_python_libs.python_lib_1 import say_it_works
from my_python_libs.python_lib_2 import  libpy2
from my_other_python_libs.other_python_lib_1 import say_it_again
from my_other_python_libs.other_python_lib_2 import libpy3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

class TestImport2(Node):

    def __init__(self):
        super().__init__('test_import_2')
        say_it_again()
        say_it_works()
        lib_2=libpy2()
        lib_3=libpy3()


        lib_2.say_it_too(self)
        lib_2.calculate(self)
        
        lib_3.say_it_too(self)
        lib_3.calculate(self)
      

    

def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_publisher = TestImport2()

            rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
