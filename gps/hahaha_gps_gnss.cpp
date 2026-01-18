//---------------------------------------------------------------------------

#pragma hdrstop
//---------------------------------------------------------------------------
#include <utility>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
//---------------------------------------------------------------------------
#include "hahaha_gps_gnss.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)

//---------------------------------------------------------------------------
namespace hahahalib
{
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
hahaha_gps_gnss::hahaha_gps_gnss()
{
	Reset();
}
//---------------------------------------------------------------------------
hahaha_gps_gnss::~hahaha_gps_gnss()
{

}
//---------------------------------------------------------------------------
hahaha_gps_gnss::hahaha_gps_gnss(const hahaha_gps_gnss& hgg)
{
    Reset();
	Copy(hgg);
}
//---------------------------------------------------------------------------
hahaha_gps_gnss::hahaha_gps_gnss(hahaha_gps_gnss&& hgg) noexcept
{
    Move(std::move(hgg));
}
//---------------------------------------------------------------------------
hahaha_gps_gnss& hahaha_gps_gnss::operator=(const hahaha_gps_gnss& hgg)
{
	Copy(hgg);

	return *this;
}
//---------------------------------------------------------------------------
hahaha_gps_gnss& hahaha_gps_gnss::operator=(hahaha_gps_gnss&& hgg) noexcept
{
	if (this != &hgg)
    {
        Move(std::move(hgg));
    }

	return *this;
}
//---------------------------------------------------------------------------
void hahaha_gps_gnss::Copy(const hahaha_gps_gnss& hgg)
{

}
//---------------------------------------------------------------------------
void hahaha_gps_gnss::Move(hahaha_gps_gnss&& hgg) noexcept
{

}
//---------------------------------------------------------------------------
int hahaha_gps_gnss::Reset()
{


	return 0;
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
bool hahaha_gps_gnss::Is_Digit(wchar_t c) const
{
	return (c >= L'0') && (c <= L'9');
}
//---------------------------------------------------------------------------
std::size_t hahaha_gps_gnss::Find_Char(const std::wstring& s, std::size_t pos, wchar_t c) const
{
	const std::size_t n = s.size();
	for (std::size_t i = pos; i < n; ++i)
	{
		const wchar_t ch = s[i];
		if (ch == c) return i;
		if (ch == L'\r' || ch == L'\n') return std::wstring::npos;
	}
	return std::wstring::npos;
}
//---------------------------------------------------------------------------
bool hahaha_gps_gnss::Parse_U32_Fixed_Digits(const std::wstring& s, std::size_t& pos, std::size_t end, int digits, std::uint32_t& out) const
{
	if (end < pos) return false;
	if (end - pos < static_cast<std::size_t>(digits)) return false;
	std::uint32_t value = 0;
	for (int i = 0; i < digits; ++i)
	{
		wchar_t c = s[pos++];
		if (!Is_Digit(c)) return false;
		value = value * 10u + static_cast<std::uint32_t>(c - L'0');
	}
	out = value;
	return true;
}
//---------------------------------------------------------------------------
bool hahaha_gps_gnss::Parse_Minutes_Scaled_1e7(const std::wstring& s, std::size_t& pos, std::size_t end, std::int64_t& minutes_scaled_1e7) const
{
	// NMEA: mm[.ffff...]
	std::uint32_t minutes_int = 0;
	if (!Parse_U32_Fixed_Digits(s, pos, end, 2, minutes_int)) return false;

	std::int64_t scaled = static_cast<std::int64_t>(minutes_int) * 10000000LL;

	if (pos < end && s[pos] == L'.')
	{
		++pos;
		std::int64_t frac = 0;
		int frac_digits = 0;
		while (pos < end && Is_Digit(s[pos]) && frac_digits < 7)
		{
			frac = (frac * 10) + (s[pos] - L'0');
			++pos;
			++frac_digits;
		}

		// Optional rounding by the 8th digit.
		bool round_up = false;
		if (pos < end && Is_Digit(s[pos]))
		{
			round_up = (s[pos] >= L'5');
			while (pos < end && Is_Digit(s[pos])) ++pos;
		}

		static constexpr std::int64_t pow10[8] = {
			1LL, 10LL, 100LL, 1000LL, 10000LL, 100000LL, 1000000LL, 10000000LL
		};
		if (frac_digits < 7) frac *= pow10[7 - frac_digits];
		if (round_up) ++frac;

		// carry into integer minutes
		if (frac >= 10000000LL)
		{
			frac -= 10000000LL;
			scaled += 10000000LL;
		}

		scaled += frac;
	}

	if (pos != end) return false;
	minutes_scaled_1e7 = scaled;
	return true;
}
//---------------------------------------------------------------------------
bool hahaha_gps_gnss::Parse_Coord_Ddmm_To_E7(const std::wstring& s, std::size_t start, std::size_t end, int deg_digits, std::int32_t& out_e7_abs) const
{
	// latitude: ddmm.mmmm, longitude: dddmm.mmmm
	if (end <= start) return false;
	if (end > s.size()) return false;
	std::size_t pos = start;

	std::uint32_t deg = 0;
	if (!Parse_U32_Fixed_Digits(s, pos, end, deg_digits, deg)) return false;

	std::int64_t minutes_scaled_1e7 = 0;
	if (!Parse_Minutes_Scaled_1e7(s, pos, end, minutes_scaled_1e7)) return false;

	const std::int64_t deg_e7 = static_cast<std::int64_t>(deg) * 10000000LL;
	const std::int64_t min_to_deg_e7 = (minutes_scaled_1e7 + 30) / 60; // rounded
	const std::int64_t coord_e7 = deg_e7 + min_to_deg_e7;

	if (coord_e7 > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) return false;
	out_e7_abs = static_cast<std::int32_t>(coord_e7);
	return true;
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
bool hahaha_gps_gnss::Parse_Rmc_Lat_Lon_E7(const std::wstring& nmea, std::int32_t& latitude_e7, std::int32_t& longitude_e7) const
{
	if (nmea.size() < 12) return false; // minimal "$??RMC,,,,,,,"

	// Accept: $GNRMC,... or $GMRMC,... etc (any 2-letter talker).
	if (nmea[0] != L'$') return false;
	if (nmea.size() < 7) return false;
	if (nmea[3] != L'R' || nmea[4] != L'M' || nmea[5] != L'C' || nmea[6] != L',') return false;

	// Field 1: time (skip)
	std::size_t p = 7;
	std::size_t comma = Find_Char(nmea, p, L',');
	if (comma == std::wstring::npos) return false;

	// Field 2: status
	p = comma + 1;
	comma = Find_Char(nmea, p, L',');
	if (comma == std::wstring::npos) return false;
	if (p >= comma) return false;
	const wchar_t status = nmea[p];
	if (status != L'A') return false;

	// Field 3: latitude
	p = comma + 1;
	comma = Find_Char(nmea, p, L',');
	if (comma == std::wstring::npos || comma == p) return false;
	std::int32_t lat_abs_e7 = 0;
	if (!Parse_Coord_Ddmm_To_E7(nmea, p, comma, 2, lat_abs_e7)) return false;

	// Field 4: N/S
	p = comma + 1;
	comma = Find_Char(nmea, p, L',');
	if (comma == std::wstring::npos) return false;
	if (p >= comma) return false;
	const wchar_t ns = nmea[p];
	if (!((ns == L'N') || (ns == L'S'))) return false;

	// Field 5: longitude
	p = comma + 1;
	comma = Find_Char(nmea, p, L',');
	if (comma == std::wstring::npos || comma == p) return false;
	std::int32_t lon_abs_e7 = 0;
	if (!Parse_Coord_Ddmm_To_E7(nmea, p, comma, 3, lon_abs_e7)) return false;

	// Field 6: E/W
	p = comma + 1;
	if (p >= nmea.size()) return false;
	const wchar_t ew = nmea[p];
	if (!((ew == L'E') || (ew == L'W'))) return false;

	latitude_e7 = (ns == L'S') ? -lat_abs_e7 : lat_abs_e7;
	longitude_e7 = (ew == L'W') ? -lon_abs_e7 : lon_abs_e7;
	return true;
}
//---------------------------------------------------------------------------
bool hahaha_gps_gnss::Parse_Rmc_Lat_Lon(const std::wstring& nmea, double& latitude_deg, double& longitude_deg) const
{
	std::int32_t lat_e7 = 0;
	std::int32_t lon_e7 = 0;
	if (!Parse_Rmc_Lat_Lon_E7(nmea, lat_e7, lon_e7)) return false;
	latitude_deg = static_cast<double>(lat_e7) * 1e-7;
	longitude_deg = static_cast<double>(lon_e7) * 1e-7;
	return true;
}
//---------------------------------------------------------------------------
std::tuple<bool, double, double> hahaha_gps_gnss::Parse_Rmc_Lat_Lon(const std::wstring& nmea) const
{
	double lat = 0.0;
	double lon = 0.0;
	const bool valid = Parse_Rmc_Lat_Lon(nmea, lat, lon);
	return std::make_tuple(valid, lat, lon);
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
} // hahahalib
//---------------------------------------------------------------------------
