from pydantic import BaseModel


class JoystickModel(BaseModel):
    x: float = None
    y: float = None
    yaw: float = None
