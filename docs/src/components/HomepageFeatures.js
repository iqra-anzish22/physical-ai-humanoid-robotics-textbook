import React from "react";
import styles from "./HomepageFeatures.module.css";

const FeatureList = [
  {
    title: "Interactive Learning",
    description: "Engage with AI & Robotics concepts through interactive tutorials and examples.",
    Img: require("@site/static/img/undraw_docusaurus_mountain.svg").default,
  },
  {
    title: "Focus on Practical Skills",
    description: "Learn the essential skills for building AI models and humanoid robots with step-by-step guidance.",
    Img: require("@site/static/img/undraw_docusaurus_tree.svg").default,
  },
  {
    title: "Powered by Modern Tech",
    description: "Explore AI, Machine Learning, and Robotics concepts using React-based interactive components.",
    Img: require("@site/static/img/undraw_docusaurus_react.svg").default,
  },
];

function Feature({ title, description, Img }) {
  return (
    <div className={styles.card}>
      <div className={styles.glow}></div>

      <div className={styles.imageWrapper}>
        <Img className={styles.image} alt={title} />
      </div>

      <h3 className={styles.title}>{title}</h3>
      <p className={styles.text}>{description}</p>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.section}>
      <div className={styles.container}>
        {FeatureList.map((feat, idx) => (
          <Feature key={idx} {...feat} />
        ))}
      </div>
    </section>
  );
}
