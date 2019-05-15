#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from rosbridge_library.internal import ros_loader
import inspect

# Keep track of atomic types and special types
atomics = ['bool', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string']
specials = ['time', 'duration']

class TypeDefDict(dict):
    FIELDS = ['fieldnames', 'fieldtypes', 'fieldarraylen', 'examples', 'constnames', 'constvalues'] 
    def __init__(self):
        dict.__init__(self)
        self['type'] = ' '
        for field in TypeDefDict.FIELDS:
            self[field] = []

def get_typedef(type_):
    """ A typedef is a dict containing the following fields:
         - string type
         - string[] fieldnames
         - string[] fieldtypes
         - int[] fieldarraylen
         - string[] examples
         - string[] constnames
         - string[] constvalues
    get_typedef will return a typedef dict for the specified message type """
    if type_ in atomics:
        # Atomics don't get a typedef
        return None

    if type_ in specials:
        # Specials get their type def mocked up
        return _get_special_typedef(type_)

    # Fetch an instance and return its typedef
    instance = ros_loader.get_message_instance(type_)
    return _get_typedef(instance)

def get_service_request_typedef(servicetype):
    """ Returns a typedef dict for the service request class for the specified service type """
    # Get an instance of the service request class and return its typedef
    instance = ros_loader.get_service_request_instance(servicetype)
    return _get_typedef(instance)

def get_service_response_typedef(servicetype):
    """ Returns a typedef dict for the service response class for the specified service type """
    # Get an instance of the service response class and return its typedef
    instance = ros_loader.get_service_response_instance(servicetype)
    return _get_typedef(instance)

def get_typedef_recursive(type_):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Just go straight into the recursive method
    return _get_typedefs_recursive(type_, [])

def get_service_request_typedef_recursive(servicetype):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Get an instance of the service request class and get its typedef
    instance = ros_loader.get_service_request_instance(servicetype)
    typedef = _get_typedef(instance)

    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])

def get_service_response_typedef_recursive(servicetype):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Get an instance of the service response class and get its typedef
    instance = ros_loader.get_service_response_instance(servicetype)
    typedef = _get_typedef(instance)

    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])

def _get_typedef(instance):
    """ Gets a typedef dict for the specified instance """
    if instance is None or not hasattr(instance, "__slots__") or not hasattr(instance, "_slot_types"):
        return None

    fieldnames = []
    fieldtypes = []
    fieldarraylen = []
    examples = []
    constnames = []
    constvalues = []
    for i in range(len(instance.__slots__)):
        # Pull out the name
        name = instance.__slots__[i]
        fieldnames.append(name)

        # Pull out the type and determine whether it's an array
        field_type = instance._slot_types[i]
        arraylen = -1
        if field_type[-1:]==']':
            if field_type[-2:-1]=='[':
                arraylen = 0
                field_type = field_type[:-2]
            else:
                split = field_type.find('[')
                arraylen = int(field_type[split+1:-1])
                field_type = field_type[:split]
        fieldarraylen.append(arraylen)

        # Get the fully qualified type
        field_instance = getattr(instance, name)
        fieldtypes.append(_type_name(field_type, field_instance))

        # Set the example as appropriate
        example = field_instance
        if arraylen>=0:
            example = []
        elif field_type not in atomics:
            example = {}
        examples.append(str(example))

    allattributes = inspect.getmembers(instance, lambda a:not(inspect.isroutine(a)))
    attributesfiltered = [a for a in allattributes if not(a[0].startswith('__') and a[0].endswith('__'))]
    for j in range(len(attributesfiltered)):
        if attributesfiltered[j][0] not in ['_md5sum','_has_header','_type','_full_text','_slot_types'] and attributesfiltered[j][0] not in instance.__slots__:
            constnames.append(str(attributesfiltered[j][0]))
            constvalues.append(str(attributesfiltered[j][1]))
    typedef = TypeDefDict()
    typedef["type"] = _type_name_from_instance(instance)
    typedef["fieldnames"] = fieldnames
    typedef["fieldtypes"] = fieldtypes
    typedef["fieldarraylen"] = fieldarraylen
    typedef["examples"] = examples
    typedef["constnames"] = constnames
    typedef["constvalues"] = constvalues

    return typedef

def _get_special_typedef(type_):
    example = TypeDefDict()
    if type_ in specials:
        example = TypeDefDict()
        example["type"] = type_
        example["fieldnames"] = ["secs", "nsecs"]
        example["fieldtypes"] = ["int32", "int32"]
        example["fieldarraylen"] = [-1, -1]
        example["examples"] = ["0", "0"]

    return example

def _get_typedefs_recursive(type_, typesseen):
    """ returns the type def for this type as well as the type defs for any fields within the type """
    if type_ in typesseen:
        # Don't put a type if it's already been seen
        return []

    # Note that we have now seen this type
    typesseen.append(type_)

    # Get the typedef for this type and make sure it's not None
    typedef = get_typedef(type_)

    return _get_subtypedefs_recursive(typedef, typesseen)

def _get_subtypedefs_recursive(typedef, typesseen):
    if typedef is None:
        return []

    # Create the list of subtypes and get the typedefs for fields
    typedefs = [ typedef ]
    for fieldtype in typedef["fieldtypes"]:
        typedefs = typedefs + _get_typedefs_recursive(fieldtype, typesseen)

    return typedefs

def _type_name(type_, instance):
    """ given a short type, and an object instance of that type,
    determines and returns the fully qualified type """
    # The fully qualified type of atomic and special types is just their original name
    if type_ in atomics or type_ in specials:
        return type_

    # If the instance is a list, then we can get no more information from the instance.
    # However, luckily, the 'type' field for list types is usually already inflated to the full type.
    if isinstance(instance, list):
        return type_

    # Otherwise, the type will come from the module and class name of the instance
    return _type_name_from_instance(instance)

def _type_name_from_instance(instance):
    mod = instance.__module__
    type_ = mod[0:mod.find('.')]+"/"+instance.__class__.__name__
    return type_

