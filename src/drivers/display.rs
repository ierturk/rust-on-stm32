use embedded_graphics_core::{pixelcolor::Rgb565, prelude::*, primitives::Rectangle};

pub struct LtdcDisplay {
    fb_ptr: *mut u16,
    width: usize,
    height: usize,
}

impl LtdcDisplay {
    pub fn new(fb_ptr: *mut u16, width: usize, height: usize) -> LtdcDisplay {
        return LtdcDisplay {
            fb_ptr,
            width,
            height,
        };
    }
}

impl OriginDimensions for LtdcDisplay {
    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}

impl DrawTarget for LtdcDisplay {
    type Error = core::convert::Infallible;
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let fb = unsafe { core::slice::from_raw_parts_mut(self.fb_ptr, self.width * self.height) };

        for Pixel(point, color) in pixels {
            if self.bounding_box().contains(point) {
                let x = point.x as usize;
                let y = point.y as usize;

                let addr: usize = (self.height - y) + self.height * x;
                fb[addr] = color.into_storage();
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_contiguous(area, core::iter::repeat(color))
    }
}
