// use font_kit::source::SystemSource;
// use font_kit::properties::Properties;
// use raqote::{DrawOptions, DrawTarget, PathBuilder, SolidSource};

fn main() {
    // // Load the desired font (Consolas in this case)
    // let font = SystemSource::new()
    //     .select_by_postscript_name("Consolas")
    //     .unwrap()
    //     .load()
    //     .unwrap();

    // // Create a DrawTarget to draw the character
    // let mut dt = DrawTarget::new(100, 100);

    // // Character to draw (in this case, 'A')
    // let character = 'A';

    // // Create a PathBuilder to construct the character's outline
    // let mut path_builder = PathBuilder::new();
    // font.layout_glyphs(
    //     Properties::new().size(100.0),
    //     &[character],
    //     |_, _, glyph| {
    //         glyph.draw(|x, y, _color| {
    //             // Print x and y positions
    //             println!("x: {}, y: {}", x, y);
                
    //             path_builder.move_to(x as f32, y as f32);
    //             SolidSource::from(raqote::Color::new(0, 0, 0, 255)).fill(
    //                 &mut dt,
    //                 &path_builder.finish(),
    //                 &DrawOptions::new(),
    //                 None,
    //             );
    //             path_builder = PathBuilder::new();
    //         });
    //     },
    // );
    
    // Save the rendered outline to an image or perform further operations
    // dt.write_png("character_outline.png").unwrap();
}