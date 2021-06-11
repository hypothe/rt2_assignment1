.. rt2_assignment1: Action documentation master file, created by
   sphinx-quickstart on Fri Jun 11 14:05:50 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to rt2_assignment1: Action's documentation!
===================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

rt2_assignment1: Action documentation
***************************************************

Implementation of the goToPoint for a non-holonomic robot using actions.

GoToPoint Module
==================
.. automodule:: scripts.go_to_point
    :members:

Position Service
==================
.. doxygenfile:: position_service.cpp
    :project: rt2_assignment1: Action
    
State Machine
==================
.. doxygenfile:: state_machine.cpp
    :project: rt2_assignment1: Action

