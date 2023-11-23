use embedded_graphics_core::{pixelcolor::Rgb565, prelude::*, primitives::Rectangle};

pub struct LtdcDisplay {
    fb_ptr: *mut u8,
    len: usize,
}

impl LtdcDisplay {
    pub fn new(fb_ptr: *mut u8, len: usize) -> LtdcDisplay {
        return LtdcDisplay { fb_ptr, len };
    }
}

impl OriginDimensions for LtdcDisplay {
    fn size(&self) -> Size {
        Size::new(240 as u32, 320 as u32)
    }
}

impl DrawTarget for LtdcDisplay {
    type Error = core::convert::Infallible;
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let fb = unsafe { core::slice::from_raw_parts_mut(self.fb_ptr, self.len) };

        for Pixel(point, color) in pixels {
            if self.bounding_box().contains(point) {
                let x = point.x as usize;
                let y = point.y as usize;

                let addr: usize = 3 * x + 3 * 240 * y;

                fb[addr] = color.g();
                fb[addr + 1] = color.b();
                fb[addr + 2] = color.r();
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_contiguous(area, core::iter::repeat(color))
    }
}
