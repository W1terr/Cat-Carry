#pragma once

// Small vector / rotation helpers on top of the game's NiPoint3 / NiMatrix3.
// NiMatrix3 uses column vectors: world = R * local, the columns are the local axes in the parent's space.
namespace M
{
	using RE::NiMatrix3;
	using RE::NiPoint3;
	using RE::NiTransform;

	inline constexpr float kPi = std::numbers::pi_v<float>;
	inline constexpr float kDegrees = kPi / 180.0f;

	inline float Clamp01(float a_value) { return std::clamp(a_value, 0.0f, 1.0f); }

	inline float Smooth(float a_t)
	{
		const float t = Clamp01(a_t);
		return t * t * (3.0f - 2.0f * t);
	}

	// frame-rate independent move of a_value towards a_target (a_rate = 1/s)
	inline float Follow(float a_value, float a_target, float a_rate, float a_delta)
	{
		return a_value + (a_target - a_value) * (1.0f - std::exp(-a_rate * a_delta));
	}

	inline float Dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

	inline NiPoint3 Cross(const NiPoint3& a, const NiPoint3& b)
	{
		return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
	}

	inline float Length(const NiPoint3& a) { return std::sqrt(Dot(a, a)); }

	inline NiPoint3 Normalized(const NiPoint3& a, const NiPoint3& a_fallback = { 0.0f, 0.0f, 1.0f })
	{
		const float length = Length(a);
		return length > 1e-6f ? a * (1.0f / length) : a_fallback;
	}

	inline NiPoint3 Lerp(const NiPoint3& a, const NiPoint3& b, float t) { return a + (b - a) * t; }

	// part of a perpendicular to the unit vector a_axis
	inline NiPoint3 Perpendicular(const NiPoint3& a, const NiPoint3& a_axis) { return a - a_axis * Dot(a, a_axis); }

	inline NiPoint3 Column(const NiMatrix3& m, int c) { return { m.entry[0][c], m.entry[1][c], m.entry[2][c] }; }

	inline NiMatrix3 FromColumns(const NiPoint3& x, const NiPoint3& y, const NiPoint3& z)
	{
		NiMatrix3 m;
		m.entry[0][0] = x.x;
		m.entry[1][0] = x.y;
		m.entry[2][0] = x.z;
		m.entry[0][1] = y.x;
		m.entry[1][1] = y.y;
		m.entry[2][1] = y.z;
		m.entry[0][2] = z.x;
		m.entry[1][2] = z.y;
		m.entry[2][2] = z.z;
		return m;
	}

	inline NiMatrix3 Identity() { return FromColumns({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }); }

	// right handed rotation of a_angle radians about the unit vector a_axis
	inline NiMatrix3 AxisAngle(const NiPoint3& a_axis, float a_angle)
	{
		const NiPoint3 k = Normalized(a_axis);
		const float    c = std::cos(a_angle);
		const float    s = std::sin(a_angle);
		const float    t = 1.0f - c;
		NiMatrix3      m;
		m.entry[0][0] = c + t * k.x * k.x;
		m.entry[0][1] = t * k.x * k.y - s * k.z;
		m.entry[0][2] = t * k.x * k.z + s * k.y;
		m.entry[1][0] = t * k.x * k.y + s * k.z;
		m.entry[1][1] = c + t * k.y * k.y;
		m.entry[1][2] = t * k.y * k.z - s * k.x;
		m.entry[2][0] = t * k.x * k.z - s * k.y;
		m.entry[2][1] = t * k.y * k.z + s * k.x;
		m.entry[2][2] = c + t * k.z * k.z;
		return m;
	}

	// smallest rotation turning the direction a into the direction b
	inline NiMatrix3 Between(const NiPoint3& a_from, const NiPoint3& a_to)
	{
		const NiPoint3 a = Normalized(a_from);
		const NiPoint3 b = Normalized(a_to);
		const NiPoint3 axis = Cross(a, b);
		const float    sine = Length(axis);
		const float    cosine = std::clamp(Dot(a, b), -1.0f, 1.0f);
		if (sine < 1e-6f) {
			if (cosine > 0.0f) {
				return Identity();
			}
			NiPoint3 other = std::abs(a.x) < 0.9f ? NiPoint3{ 1, 0, 0 } : NiPoint3{ 0, 1, 0 };
			return AxisAngle(Normalized(Cross(a, other)), kPi);
		}
		return AxisAngle(axis * (1.0f / sine), std::atan2(sine, cosine));
	}

	// orthonormal frame: x = a, y = b made perpendicular to a, z = x * y
	inline NiMatrix3 Frame(const NiPoint3& a, const NiPoint3& b)
	{
		const NiPoint3 x = Normalized(a);
		NiPoint3       y = Perpendicular(b, x);
		if (Length(y) < 1e-5f) {
			y = Perpendicular(std::abs(x.z) < 0.9f ? NiPoint3{ 0, 0, 1 } : NiPoint3{ 1, 0, 0 }, x);
		}
		y = Normalized(y);
		return FromColumns(x, y, Cross(x, y));
	}

	// rotation that turns the local directions a, b into the world directions a, b (a exactly, b as close as it can)
	inline NiMatrix3 Align(const NiPoint3& a_localA, const NiPoint3& a_localB, const NiPoint3& a_worldA, const NiPoint3& a_worldB)
	{
		return Frame(a_worldA, a_worldB) * Frame(a_localA, a_localB).Transpose();
	}

	struct Quat
	{
		float w{ 1.0f };
		float x{ 0.0f };
		float y{ 0.0f };
		float z{ 0.0f };
	};

	inline Quat ToQuat(const NiMatrix3& m)
	{
		Quat        q;
		const float trace = m.entry[0][0] + m.entry[1][1] + m.entry[2][2];
		if (trace > 0.0f) {
			const float s = std::sqrt(trace + 1.0f) * 2.0f;
			q.w = 0.25f * s;
			q.x = (m.entry[2][1] - m.entry[1][2]) / s;
			q.y = (m.entry[0][2] - m.entry[2][0]) / s;
			q.z = (m.entry[1][0] - m.entry[0][1]) / s;
		} else if (m.entry[0][0] > m.entry[1][1] && m.entry[0][0] > m.entry[2][2]) {
			const float s = std::sqrt(1.0f + m.entry[0][0] - m.entry[1][1] - m.entry[2][2]) * 2.0f;
			q.w = (m.entry[2][1] - m.entry[1][2]) / s;
			q.x = 0.25f * s;
			q.y = (m.entry[0][1] + m.entry[1][0]) / s;
			q.z = (m.entry[0][2] + m.entry[2][0]) / s;
		} else if (m.entry[1][1] > m.entry[2][2]) {
			const float s = std::sqrt(1.0f + m.entry[1][1] - m.entry[0][0] - m.entry[2][2]) * 2.0f;
			q.w = (m.entry[0][2] - m.entry[2][0]) / s;
			q.x = (m.entry[0][1] + m.entry[1][0]) / s;
			q.y = 0.25f * s;
			q.z = (m.entry[1][2] + m.entry[2][1]) / s;
		} else {
			const float s = std::sqrt(1.0f + m.entry[2][2] - m.entry[0][0] - m.entry[1][1]) * 2.0f;
			q.w = (m.entry[1][0] - m.entry[0][1]) / s;
			q.x = (m.entry[0][2] + m.entry[2][0]) / s;
			q.y = (m.entry[1][2] + m.entry[2][1]) / s;
			q.z = 0.25f * s;
		}
		return q;
	}

	inline NiMatrix3 ToMatrix(const Quat& a_q)
	{
		const float length = std::sqrt(a_q.w * a_q.w + a_q.x * a_q.x + a_q.y * a_q.y + a_q.z * a_q.z);
		const float inv = length > 1e-8f ? 1.0f / length : 1.0f;
		const float w = a_q.w * inv, x = a_q.x * inv, y = a_q.y * inv, z = a_q.z * inv;
		NiMatrix3   m;
		m.entry[0][0] = 1 - 2 * (y * y + z * z);
		m.entry[0][1] = 2 * (x * y - z * w);
		m.entry[0][2] = 2 * (x * z + y * w);
		m.entry[1][0] = 2 * (x * y + z * w);
		m.entry[1][1] = 1 - 2 * (x * x + z * z);
		m.entry[1][2] = 2 * (y * z - x * w);
		m.entry[2][0] = 2 * (x * z - y * w);
		m.entry[2][1] = 2 * (y * z + x * w);
		m.entry[2][2] = 1 - 2 * (x * x + y * y);
		return m;
	}

	inline Quat Slerp(const Quat& a, Quat b, float t)
	{
		float cosine = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
		if (cosine < 0.0f) {
			b = { -b.w, -b.x, -b.y, -b.z };
			cosine = -cosine;
		}
		float ka = 1.0f - t;
		float kb = t;
		if (cosine < 0.9995f) {
			const float angle = std::acos(std::clamp(cosine, -1.0f, 1.0f));
			const float sine = std::sin(angle);
			ka = std::sin((1.0f - t) * angle) / sine;
			kb = std::sin(t * angle) / sine;
		}
		return { ka * a.w + kb * b.w, ka * a.x + kb * b.x, ka * a.y + kb * b.y, ka * a.z + kb * b.z };
	}

	inline NiMatrix3 Slerp(const NiMatrix3& a, const NiMatrix3& b, float t)
	{
		if (t <= 0.0f) {
			return a;
		}
		if (t >= 1.0f) {
			return b;
		}
		return ToMatrix(Slerp(ToQuat(a), ToQuat(b), t));
	}

	// rotation angle of a rotation matrix (0..pi)
	inline float Angle(const NiMatrix3& r)
	{
		const float trace = r.entry[0][0] + r.entry[1][1] + r.entry[2][2];
		return std::acos(std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f));
	}

	// the same rotation, but at most a_max radians
	inline NiMatrix3 Limit(const NiMatrix3& r, float a_max)
	{
		const float angle = Angle(r);
		return angle > a_max ? Slerp(Identity(), r, a_max / angle) : r;
	}

	// twist part (radians) of the rotation r about the unit vector a_axis
	inline float Twist(const NiMatrix3& r, const NiPoint3& a_axis)
	{
		const Quat q = ToQuat(r);
		float      along = q.x * a_axis.x + q.y * a_axis.y + q.z * a_axis.z;
		float      w = q.w;
		if (w < 0.0f) {
			w = -w;
			along = -along;
		}
		return 2.0f * std::atan2(along, w);
	}

	inline float WrapAngle(float a)
	{
		while (a > kPi) {
			a -= 2.0f * kPi;
		}
		while (a < -kPi) {
			a += 2.0f * kPi;
		}
		return a;
	}

	inline float LerpAngle(float a, float b, float t) { return a + WrapAngle(b - a) * t; }

	inline bool Same(const NiMatrix3& a, const NiMatrix3& b)
	{
		return std::memcmp(&a, &b, sizeof(NiMatrix3)) == 0;
	}

	// local transform below a_parent that puts a node at the world transform a_world
	inline NiTransform ToLocal(const NiTransform& a_parent, const NiTransform& a_world)
	{
		return a_parent.Invert() * a_world;
	}

	// rotation of a reference whose heading (angle.z) is a_heading: +Y forward = (sin, cos, 0)
	inline NiMatrix3 Heading(float a_heading)
	{
		const float s = std::sin(a_heading);
		const float c = std::cos(a_heading);
		return FromColumns({ c, -s, 0.0f }, { s, c, 0.0f }, { 0.0f, 0.0f, 1.0f });
	}
}
