from pydantic import BaseModel


class JoystickModel(BaseModel):
    x: float = None
    y: float = None

class GaitModel(BaseModel):
    is_run: bool = False

class PlatformHeightModel(BaseModel):
    height: int = 2