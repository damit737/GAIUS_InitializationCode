#include <gui/common/FrontendApplication.hpp>

FrontendApplication::FrontendApplication(Model& m, FrontendHeap& heap)
    : FrontendApplicationBase(m, heap)
{
	uint16_t* const cacheStartAddr = (uint16_t*)0xD0000000;
	const uint32_t cacheSize = 0x500000;
	touchgfx::Bitmap::removeCache();
	Bitmap::setCache(cacheStartAddr, cacheSize,256);
}
