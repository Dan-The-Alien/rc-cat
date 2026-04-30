from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDConfig:
	kp: float
	ki: float
	kd: float
	target: float = 0.0
	output_min: float = -1.0
	output_max: float = 1.0
	integral_min: float = -1.0
	integral_max: float = 1.0


@dataclass
class PIDResult:
	measurement: float
	target: float
	error: float
	output: float


class PIDController:
	"""Simple normalized PID controller with clamped output."""

	def __init__(self, config: PIDConfig):
		self.config = config
		self.integral = 0.0
		self.previous_error: float | None = None
		self.last_output = 0.0

	def reset(self) -> None:
		self.integral = 0.0
		self.previous_error = None
		self.last_output = 0.0

	def configure(self, config: PIDConfig) -> None:
		self.config = config
		self.reset()

	def step(self, measurement: float, dt: float) -> PIDResult:
		if dt <= 0:
			dt = 1e-6

		error = self.config.target - measurement
		self.integral += error * dt
		self.integral = _clamp(self.integral, self.config.integral_min, self.config.integral_max)

		if self.previous_error is None:
			derivative = 0.0
		else:
			derivative = (error - self.previous_error) / dt

		output = (
			(self.config.kp * error)
			+ (self.config.ki * self.integral)
			+ (self.config.kd * derivative)
		)
		output = _clamp(output, self.config.output_min, self.config.output_max)

		self.previous_error = error
		self.last_output = output
		return PIDResult(
			measurement=measurement,
			target=self.config.target,
			error=error,
			output=output,
		)


def _clamp(value: float, low: float, high: float) -> float:
	return max(low, min(high, value))
