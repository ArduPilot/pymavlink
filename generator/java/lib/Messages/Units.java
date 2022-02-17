package com.MAVLink.Messages;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
@SuppressWarnings("unchecked")
public @interface Units {

	String value();

	class Test {

		@Units("m/s")
		public float speed;

		public static void main(String[] args) throws Exception {
			Field f = Test.class.getField("speed");
			Units anno = (Units) f.getAnnotation(Units.class);
			System.out.println(UnitsEnum.fromName(anno.value()));
		}
	}
}
