dumbo_hardware_test
===================

To run:

   1. Make sure the CAN drivers are loaded:
      
      sudo pcicanII.sh start
      sudo pcican.sh start

   2. Run the text UI for low level comm & control of the Schunk modules of Dumbo's arms:
      
      rosrun dumbo_arm_test dumbo_arm_test