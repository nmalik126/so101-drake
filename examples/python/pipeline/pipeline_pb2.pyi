from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Request(_message.Message):
    __slots__ = ("id",)
    ID_FIELD_NUMBER: _ClassVar[int]
    id: str
    def __init__(self, id: _Optional[str] = ...) -> None: ...

class Response(_message.Message):
    __slots__ = ("id", "success")
    ID_FIELD_NUMBER: _ClassVar[int]
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    id: str
    success: bool
    def __init__(self, id: _Optional[str] = ..., success: bool = ...) -> None: ...

class ImagePair(_message.Message):
    __slots__ = ("depth_data", "color_data")
    DEPTH_DATA_FIELD_NUMBER: _ClassVar[int]
    COLOR_DATA_FIELD_NUMBER: _ClassVar[int]
    depth_data: bytes
    color_data: bytes
    def __init__(self, depth_data: _Optional[bytes] = ..., color_data: _Optional[bytes] = ...) -> None: ...

class ImagePairRequest(_message.Message):
    __slots__ = ("request", "imagePair")
    REQUEST_FIELD_NUMBER: _ClassVar[int]
    IMAGEPAIR_FIELD_NUMBER: _ClassVar[int]
    request: Request
    imagePair: ImagePair
    def __init__(self, request: _Optional[_Union[Request, _Mapping]] = ..., imagePair: _Optional[_Union[ImagePair, _Mapping]] = ...) -> None: ...

class ImagePairResponse(_message.Message):
    __slots__ = ("response", "imagePair")
    RESPONSE_FIELD_NUMBER: _ClassVar[int]
    IMAGEPAIR_FIELD_NUMBER: _ClassVar[int]
    response: Response
    imagePair: ImagePair
    def __init__(self, response: _Optional[_Union[Response, _Mapping]] = ..., imagePair: _Optional[_Union[ImagePair, _Mapping]] = ...) -> None: ...

class Grasp(_message.Message):
    __slots__ = ("grasp",)
    GRASP_FIELD_NUMBER: _ClassVar[int]
    grasp: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, grasp: _Optional[_Iterable[float]] = ...) -> None: ...

class GraspRequest(_message.Message):
    __slots__ = ("request", "grasp", "q_start")
    REQUEST_FIELD_NUMBER: _ClassVar[int]
    GRASP_FIELD_NUMBER: _ClassVar[int]
    Q_START_FIELD_NUMBER: _ClassVar[int]
    request: Request
    grasp: Grasp
    q_start: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, request: _Optional[_Union[Request, _Mapping]] = ..., grasp: _Optional[_Union[Grasp, _Mapping]] = ..., q_start: _Optional[_Iterable[float]] = ...) -> None: ...

class GraspResponse(_message.Message):
    __slots__ = ("response", "grasp")
    RESPONSE_FIELD_NUMBER: _ClassVar[int]
    GRASP_FIELD_NUMBER: _ClassVar[int]
    response: Response
    grasp: Grasp
    def __init__(self, response: _Optional[_Union[Response, _Mapping]] = ..., grasp: _Optional[_Union[Grasp, _Mapping]] = ...) -> None: ...

class Trajectory(_message.Message):
    __slots__ = ("traj",)
    TRAJ_FIELD_NUMBER: _ClassVar[int]
    traj: bytes
    def __init__(self, traj: _Optional[bytes] = ...) -> None: ...

class PlanTrajectories(_message.Message):
    __slots__ = ("pick_traj", "place_traj", "return_traj")
    PICK_TRAJ_FIELD_NUMBER: _ClassVar[int]
    PLACE_TRAJ_FIELD_NUMBER: _ClassVar[int]
    RETURN_TRAJ_FIELD_NUMBER: _ClassVar[int]
    pick_traj: Trajectory
    place_traj: Trajectory
    return_traj: Trajectory
    def __init__(self, pick_traj: _Optional[_Union[Trajectory, _Mapping]] = ..., place_traj: _Optional[_Union[Trajectory, _Mapping]] = ..., return_traj: _Optional[_Union[Trajectory, _Mapping]] = ...) -> None: ...

class TrajectoryRequest(_message.Message):
    __slots__ = ("request", "traj")
    REQUEST_FIELD_NUMBER: _ClassVar[int]
    TRAJ_FIELD_NUMBER: _ClassVar[int]
    request: Request
    traj: Trajectory
    def __init__(self, request: _Optional[_Union[Request, _Mapping]] = ..., traj: _Optional[_Union[Trajectory, _Mapping]] = ...) -> None: ...

class PlanTrajectoriesResponse(_message.Message):
    __slots__ = ("response", "planTrajectories")
    RESPONSE_FIELD_NUMBER: _ClassVar[int]
    PLANTRAJECTORIES_FIELD_NUMBER: _ClassVar[int]
    response: Response
    planTrajectories: PlanTrajectories
    def __init__(self, response: _Optional[_Union[Response, _Mapping]] = ..., planTrajectories: _Optional[_Union[PlanTrajectories, _Mapping]] = ...) -> None: ...
