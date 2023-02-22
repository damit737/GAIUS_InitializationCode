#include <gui/start_screen/startView.hpp>
#include "BitmapDatabase.hpp"
startView::startView()
{

}

void startView::setupScreen()
{
    startViewBase::setupScreen();
	Bitmap::cache( BITMAP_A_START_ID );
	Bitmap::cache( BITMAP_BUTTON_00_00_ID );
	Bitmap::cache( BITMAP_BUTTON_00_01_ID );
}

void startView::tearDownScreen()
{
    startViewBase::tearDownScreen();
}
