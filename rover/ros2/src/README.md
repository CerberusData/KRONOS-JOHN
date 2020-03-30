# Ros Node Readme Template 

## Short Description 

Describe here what is the main purpose of the node and how it workst at high level. You can use a diagram or something similar if you like.

## Topics/Services/Parameters/Env_vars

This section describes the topics that the node uses and for what purpose.

### Subscribers

The node is subscribed to the following topics:

* /blabla/etcetc
    * Type: yeahyeah
    * Description: It is used for blabla in etc and etc.
* ...

### Publishers      

The node publishes to the following topics:

* /blabla/etcetc
    * Type: sisasn't
    * Description: It is used to inform the status of the blabla.
* ...

### Services

The node exposes the following services:

* /node/service1
    * Description: blabla
    * Parameters: type param1, type param2
    * Returns: type returned

### Parameters

This node has the following parameters:
* /node/param: type, it is used for...
* ...

### Environment variables

The node uses these environment variables:
* ENV_VAR_NAME: type, and its usage



## Technical/Implementation details

Here should be exaplained the details in order to understand the implementations. 
* If the nodes uses threading or multiprocess for a task, the details of how it works should be explained here. 
* If some third party library is used, elaborate the usage here.
* Anything you consider may be confusing of the code could be exaplained here.

