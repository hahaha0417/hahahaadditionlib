//---------------------------------------------------------------------------

#ifndef hahaha_gps_gnssH
#define hahaha_gps_gnssH
//---------------------------------------------------------------------------
#pragma once
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#include <tuple>
#include <string>
#include <cstddef>
#include <cstdint>
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
namespace hahahalib
{
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

class hahaha_gps_gnss
{
public:
	hahaha_gps_gnss();
	~hahaha_gps_gnss();

	hahaha_gps_gnss(const hahaha_gps_gnss& hgg);
    hahaha_gps_gnss(hahaha_gps_gnss&& hgg) noexcept;
	hahaha_gps_gnss& operator=(const hahaha_gps_gnss& hgg);
	hahaha_gps_gnss& operator=(hahaha_gps_gnss&& hgg) noexcept;
	void Copy(const hahaha_gps_gnss& hgg);
    void Move(hahaha_gps_gnss&& hgg) noexcept;
public:
	int Reset();

public:
	// Parse NMEA RMC sentence ($GNRMC / $GMRMC / $GPRMC ...), return degrees * 1e7.
	// Only accepts status 'A' (valid fix). No heap allocations, no sscanf/strtok.
	bool Parse_Rmc_Lat_Lon(const std::wstring& nmea, double& latitude_deg, double& longitude_deg) const;
	std::tuple<bool, double, double> Parse_Rmc_Lat_Lon(const std::wstring& nmea) const;

private:
	bool Is_Digit(wchar_t c) const;
	std::size_t Find_Char(const std::wstring& s, std::size_t pos, wchar_t c) const;
	bool Parse_U32_Fixed_Digits(const std::wstring& s, std::size_t& pos, std::size_t end, int digits, std::uint32_t& out) const;
	bool Parse_Minutes_Scaled_1e7(const std::wstring& s, std::size_t& pos, std::size_t end, std::int64_t& minutes_scaled_1e7) const;
	bool Parse_Coord_Ddmm_To_E7(const std::wstring& s, std::size_t start, std::size_t end, int deg_digits, std::int32_t& out_e7_abs) const;
	bool Parse_Rmc_Lat_Lon_E7(const std::wstring& nmea, std::int32_t& latitude_e7, std::int32_t& longitude_e7) const;

};



//---------------------------------------------------------------------------
} // hahahalib
//---------------------------------------------------------------------------



#endif
