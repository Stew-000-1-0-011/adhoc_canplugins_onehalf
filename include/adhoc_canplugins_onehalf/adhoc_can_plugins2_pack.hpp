#pragma once

#include <cstring>
#if __cplusplus > 202002L
#include <bit>
#endif
#include <type_traits>
#include <utility>

#include <CRSLib/include/std_type.hpp>

namespace adhoc_canplugins_onehalf
{
	using namespace CRSLib::IntegerTypes;

	constexpr u32 can_mtu = 8;

#if __cplusplus > 202002L
	template<class T>
	concept BeAbleToPackC = sizeof(T) <= can_mtu && std::is_trivially_copyable_v<T>;

	template<std::endian endian, BeAbleToPackC T>
	inline void pack(void *const dest, const T& value) noexcept
	{
		std::memcpy(dest, &value, sizeof(T));

		if constexpr(std::endian::native != endian)
		{
			for(unsigned int i = 0; i < sizeof(T) / 2; ++i)
			{
				std::swap(static_cast<char *>(dest)[i], static_cast<char *>(dest)[sizeof(T) - 1 - i]);
			}
		}
	}

	template<std::endian endian, BeAbleToPackC T>
	inline T unpack(const void *const src) noexcept
	{
		if constexpr(std::endian::native != endian)
		{
			char tmp[can_mtu];
			for(unsigned int i = 0; i < sizeof(T); ++i)
			{
				tmp[i] = static_cast<const char *>(src)[sizeof(T) - 1 - i];
			}

			// 確か大丈夫だったはず(char *からは任意の型にアクセスできるんじゃないっけ？)
			return *reinterpret_cast<const T *>(tmp);
		}
		else
		{
			return *reinterpret_cast<const T *>(src);
		}
	}
#else

	template<class T>
	inline constexpr bool BeAbleToPackC = sizeof(T) <= can_mtu && std::is_trivially_copyable_v<T>;

	template<class T>
	inline void pack(void *const dest, const T& src, const bool endian_inverse = true)
	{
		static_assert(BeAbleToPackC<T>);
		
		std::memcpy(dest, &src, sizeof(T));

		if(endian_inverse)
		{
			for(unsigned int i = 0; i < sizeof(T) / 2; ++i)
			{
				std::swap(static_cast<char *>(dest)[i], static_cast<char *>(dest)[sizeof(T) - 1 - i]);
			}
		}
	}

	template<class T>
	inline T unpack(const void *const src, const bool endian_inverse) noexcept
	{
		if(endian_inverse)
		{
			char tmp[can_mtu];
			for(unsigned int i = 0; i < sizeof(T); ++i)
			{
				tmp[i] = static_cast<const char *>(src)[sizeof(T) - 1 - i];
			}

			// 確か大丈夫だったはず(char *からは任意の型にアクセスできるんじゃないっけ？)
			return *reinterpret_cast<const T *>(tmp);
		}
		else
		{
			return *reinterpret_cast<const T *>(src);
		}
	}
#endif
}
