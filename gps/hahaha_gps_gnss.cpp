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
    if(this != &hgg)
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

bool hahaha_gps_gnss::Is_Digit(wchar_t c) const
{
    return (c >= L'0') && (c <= L'9');
}
//---------------------------------------------------------------------------

std::size_t hahaha_gps_gnss::Find_Char(const std::wstring& s, std::size_t pos, wchar_t c) const
{
    const std::size_t n_ = s.size();

    for(std::size_t i = pos; i < n_; ++i)
    {
        const wchar_t ch_ = s[i];

        if(ch_ == c)
        {
            return i;
        }

        if(ch_ == L'\r' || ch_ == L'\n')
        {
            return std::wstring::npos;
        }
    }

    return std::wstring::npos;
}
//---------------------------------------------------------------------------

bool hahaha_gps_gnss::Parse_U32_Fixed_Digits(
    const std::wstring& s,
    std::size_t& pos,
    std::size_t end,
    int digits,
    std::uint32_t& out
) const
{
    if(end < pos)
    {
        return false;
    }

    if(end - pos < static_cast<std::size_t>(digits))
    {
        return false;
    }

    std::uint32_t value_ = 0;

    for(int i = 0; i < digits; ++i)
    {
        wchar_t c_ = s[pos++];

        if(!Is_Digit(c_))
        {
            return false;
        }

        value_ = value_ * 10u + static_cast<std::uint32_t>(c_ - L'0');
    }

    out = value_;
    return true;
}
//---------------------------------------------------------------------------

bool hahaha_gps_gnss::Parse_Minutes_Scaled_1e7(
    const std::wstring& s,
    std::size_t& pos,
    std::size_t end,
    std::int64_t& minutes_scaled_1e7
) const
{
    std::uint32_t minutes_int_ = 0;

    if(!Parse_U32_Fixed_Digits(s, pos, end, 2, minutes_int_))
    {
        return false;
    }

    std::int64_t scaled_ = static_cast<std::int64_t>(minutes_int_) * 10000000LL;

    if(pos < end && s[pos] == L'.')
    {
        ++pos;

        std::int64_t frac_ = 0;
        int frac_digits_ = 0;

        while(pos < end && Is_Digit(s[pos]) && frac_digits_ < 7)
        {
            frac_ = frac_ * 10 + (s[pos] - L'0');
            ++pos;
            ++frac_digits_;
        }

        bool round_up_ = false;

        if(pos < end && Is_Digit(s[pos]))
        {
            round_up_ = (s[pos] >= L'5');

            while(pos < end && Is_Digit(s[pos]))
            {
                ++pos;
            }
        }

        static constexpr std::int64_t pow10_[8] = {
            1LL, 10LL, 100LL, 1000LL,
            10000LL, 100000LL, 1000000LL, 10000000LL
        };

        if(frac_digits_ < 7)
        {
            frac_ *= pow10_[7 - frac_digits_];
        }

        if(round_up_)
        {
            ++frac_;
        }

        if(frac_ >= 10000000LL)
        {
            frac_ -= 10000000LL;
            scaled_ += 10000000LL;
        }

        scaled_ += frac_;
    }

    if(pos != end)
    {
        return false;
    }

    minutes_scaled_1e7 = scaled_;
    return true;
}
//---------------------------------------------------------------------------

bool hahaha_gps_gnss::Parse_Coord_Ddmm_To_E7(
    const std::wstring& s,
    std::size_t start,
    std::size_t end,
    int deg_digits,
    std::int32_t& out_e7_abs
) const
{
    if(end <= start)
    {
        return false;
    }

    if(end > s.size())
    {
        return false;
    }

    std::size_t pos_ = start;

    std::uint32_t deg_ = 0;

    if(!Parse_U32_Fixed_Digits(s, pos_, end, deg_digits, deg_))
    {
        return false;
    }

    std::int64_t minutes_scaled_1e7_ = 0;

    if(!Parse_Minutes_Scaled_1e7(s, pos_, end, minutes_scaled_1e7_))
    {
        return false;
    }

    const std::int64_t deg_e7_ = static_cast<std::int64_t>(deg_) * 10000000LL;
    const std::int64_t min_to_deg_e7_ = (minutes_scaled_1e7_ + 30) / 60;
    const std::int64_t coord_e7_ = deg_e7_ + min_to_deg_e7_;

    if(coord_e7_ > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))
    {
        return false;
    }

    out_e7_abs = static_cast<std::int32_t>(coord_e7_);
    return true;
}
//---------------------------------------------------------------------------

bool hahaha_gps_gnss::Parse_Rmc_Lat_Lon_E7(
    const std::wstring& nmea,
    std::int32_t& latitude_e7,
    std::int32_t& longitude_e7
) const
{
    if(nmea.size() < 12)
    {
        return false;
    }

    if(nmea[0] != L'$')
    {
        return false;
    }

    if(nmea.size() < 7)
    {
        return false;
    }

    if(nmea[3] != L'R' || nmea[4] != L'M' || nmea[5] != L'C' || nmea[6] != L',')
    {
        return false;
    }

    std::size_t p_ = 7;
    std::size_t comma_ = Find_Char(nmea, p_, L',');

    if(comma_ == std::wstring::npos)
    {
        return false;
    }

    p_ = comma_ + 1;
    comma_ = Find_Char(nmea, p_, L',');

    if(comma_ == std::wstring::npos)
    {
        return false;
    }

    if(p_ >= comma_)
    {
        return false;
    }

    const wchar_t status_ = nmea[p_];

    if(status_ != L'A')
    {
        return false;
    }

    p_ = comma_ + 1;
    comma_ = Find_Char(nmea, p_, L',');

    if(comma_ == std::wstring::npos || comma_ == p_)
    {
        return false;
    }

    std::int32_t lat_abs_e7_ = 0;

    if(!Parse_Coord_Ddmm_To_E7(nmea, p_, comma_, 2, lat_abs_e7_))
    {
        return false;
    }

    p_ = comma_ + 1;
    comma_ = Find_Char(nmea, p_, L',');

    if(comma_ == std::wstring::npos)
    {
        return false;
    }

    if(p_ >= comma_)
    {
        return false;
    }

    const wchar_t ns_ = nmea[p_];

    if(!(ns_ == L'N' || ns_ == L'S'))
    {
        return false;
    }

    p_ = comma_ + 1;
    comma_ = Find_Char(nmea, p_, L',');

    if(comma_ == std::wstring::npos || comma_ == p_)
    {
        return false;
    }

    std::int32_t lon_abs_e7_ = 0;

    if(!Parse_Coord_Ddmm_To_E7(nmea, p_, comma_, 3, lon_abs_e7_))
    {
        return false;
    }

    p_ = comma_ + 1;

    if(p_ >= nmea.size())
    {
        return false;
    }

    const wchar_t ew_ = nmea[p_];

    if(!(ew_ == L'E' || ew_ == L'W'))
    {
        return false;
    }

    latitude_e7 = (ns_ == L'S') ? -lat_abs_e7_ : lat_abs_e7_;
    longitude_e7 = (ew_ == L'W') ? -lon_abs_e7_ : lon_abs_e7_;

    return true;
}
//---------------------------------------------------------------------------

bool hahaha_gps_gnss::Parse_Rmc_Lat_Lon(
    const std::wstring& nmea,
    double& latitude_deg,
    double& longitude_deg
) const
{
    std::int32_t lat_e7_ = 0;
    std::int32_t lon_e7_ = 0;

    if(!Parse_Rmc_Lat_Lon_E7(nmea, lat_e7_, lon_e7_))
    {
        return false;
    }

    latitude_deg = static_cast<double>(lat_e7_) * 1e-7;
    longitude_deg = static_cast<double>(lon_e7_) * 1e-7;

    return true;
}
//---------------------------------------------------------------------------

std::tuple<bool, double, double> hahaha_gps_gnss::Parse_Rmc_Lat_Lon(const std::wstring& nmea) const
{
    double lat_ = 0.0;
    double lon_ = 0.0;

    const bool valid_ = Parse_Rmc_Lat_Lon(nmea, lat_, lon_);

    return std::make_tuple(valid_, lat_, lon_);
}
//---------------------------------------------------------------------------

} // namespace hahahalib
//---------------------------------------------------------------------------


